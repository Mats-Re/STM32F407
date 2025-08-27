#include "DB.h"
#include <math.h>
#include <stddef.h>
#include "stdio.h"
#include <stdlib.h>

#define NUM_POINTS 998
#define FREQ_START 1000.0f     // 1 kHz
#define FREQ_STEP  200.0f
#define THRESH_DB  3.0f        // 通常认为3dB衰减点为边界
#define MIN_VALID_BW 3000.0f   // 至少3kHz带宽才认为是带通/带阻
char detect_filter_type(float att[998])
{
    // 1. 对原始响应进行简单平滑（滑动窗口3点平均）
    float smooth_att[NUM_POINTS];
    for (int i = 1; i < NUM_POINTS - 1; ++i)
        smooth_att[i] = (att[i - 1] + att[i] + att[i + 1]) / 3.0f;
    smooth_att[0] = att[0];
    smooth_att[NUM_POINTS - 1] = att[NUM_POINTS - 1];
    // 2. 计算最大/最小值及其索引（用于中心频率分析）
    float peak_value = smooth_att[0], bottom_value = smooth_att[0];
    int peak_index = 0, bottom_index = 0;
    for (int i = 1; i < NUM_POINTS; ++i) {
        if (smooth_att[i] > peak_value) 
				{
            peak_value = smooth_att[i];
            peak_index = i;
        }
        if (smooth_att[i] < bottom_value) 
				{
            bottom_value = smooth_att[i];
            bottom_index = i;
        }
    }

    float range = peak_value - bottom_value;
    float thresh_db = range * 0.25f;  // 可调系数

    // 3. 归一化（数值越大，表示衰减越大）
    float norm_att[NUM_POINTS];
    for (int i = 0; i < NUM_POINTS; ++i)
        norm_att[i] = peak_value - smooth_att[i];

    // 4. 动态分段
    int low_end = NUM_POINTS / 5;
    int mid_start = NUM_POINTS * 2 / 5;
    int mid_end = NUM_POINTS * 3 / 5;
    int high_start = NUM_POINTS * 4 / 5;

    // 5. 排除极值后的均值（去掉前后各10%的点）
    float low_sum = 0, mid_sum = 0, high_sum = 0;
    int low_cnt = 0, mid_cnt = 0, high_cnt = 0;

    for (int i = 0; i < low_end; ++i)
        if (norm_att[i] > 0.1f * range && norm_att[i] < 0.9f * range)
            low_sum += norm_att[i], ++low_cnt;

    for (int i = mid_start; i < mid_end; ++i)
        if (norm_att[i] > 0.1f * range && norm_att[i] < 0.9f * range)
            mid_sum += norm_att[i], ++mid_cnt;

    for (int i = high_start; i < NUM_POINTS; ++i)
        if (norm_att[i] > 0.1f * range && norm_att[i] < 0.9f * range)
            high_sum += norm_att[i], ++high_cnt;

    float low_avg = low_cnt ? low_sum / low_cnt : 0;
    float mid_avg = mid_cnt ? mid_sum / mid_cnt : 0;
    float high_avg = high_cnt ? high_sum / high_cnt : 0;

    // 6. 类型判断逻辑（增强鲁棒性）
    int low_pass = (low_avg < thresh_db && high_avg > thresh_db && mid_avg < high_avg);
    int high_pass = (low_avg > thresh_db && high_avg < thresh_db && mid_avg < low_avg);
    int band_pass = (mid_avg < thresh_db && low_avg > thresh_db && high_avg > thresh_db);
    int band_stop = (mid_avg > thresh_db && low_avg < thresh_db && high_avg < thresh_db);

    // 7. 输出类型
    if (low_pass) {
        printf("type.txt=\"Lowpass\"\xff\xff\xff");
        return 'L';
    }

    if (high_pass) {
        printf("type.txt=\"Highpass\"\xff\xff\xff");
        return 'H';
    }

    // 增强带通判断：最低点在中段
    if (band_pass || (bottom_index > mid_start && bottom_index < mid_end)) {
        printf("type.txt=\"Bandpass\"\xff\xff\xff");
        return 'B';
    }

    // 增强带阻判断：最高点在中段
    if (band_stop || (peak_index > mid_start && peak_index < mid_end)) {
        printf("type.txt=\"Bandstop\"\xff\xff\xff");
        return 'N';
    }

    if (range < 3.0f) {
        printf("type.txt=\"Flat\"\xff\xff\xff");
        return 'F';
    }

    printf("type.txt=\"Unknown\"\xff\xff\xff");
    return 'U';
}



    

