#ifndef __FILTER_DETECT_H
#define __FILTER_DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_DATA_POINTS 495  // 采样点数


/**
 * @brief 根据频率响应衰减数组判断滤波器类型
 * @param att 衰减数组（单位 dB），共 FILTER_DATA_POINTS 个点
 * @return FilterType 枚举，表示滤波器类型
 */
char detect_filter_type(float att[495]);

/**
 * @brief 将滤波器类型枚举转换为字符串
 * @param type 枚举类型
 * @return const char* 对应字符串
 */

#ifdef __cplusplus
}
#endif

#endif // __FILTER_DETECT_H