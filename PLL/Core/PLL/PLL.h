#ifndef __PLL_H__
#define __PLL_H__

#include <stdint.h>

// 锁相环参数配置结构体
typedef struct {
    float k;          // SOGI增益系数 (推荐√2)
    float omega_n;    // 额定角频率(rad/s)
    float kp;         // 比例系数
    float ki;         // 积分系数
} SPLL_ConfigTypeDef;

// 锁相环状态结构体
typedef struct {
    // 配置参数
    SPLL_ConfigTypeDef config;
    
    // 状态变量
    float v[2];       // SOGI主通道状态
    float qv[2];      // SOGI正交通道状态
    float omega;      // 估计角频率(rad/s)
    float theta;      // 估计相位(rad)
    float int_err;    // 积分误差
    uint32_t counter; // 采样计数器
    
    // 中间变量缓存
    float _v_dq_d;    // d轴分量
    float _v_dq_q;    // q轴分量
} SPLL_HandleTypeDef;

// 函数原型
void SPLL_Init(SPLL_HandleTypeDef *hpll, const SPLL_ConfigTypeDef *config);
void SPLL_Update(SPLL_HandleTypeDef *hpll, float vin, float Ts);
void SPLL_Reset(SPLL_HandleTypeDef *hpll);
float SPLL_GetFrequency(SPLL_HandleTypeDef *hpll);
float SPLL_GetPhaseRad(SPLL_HandleTypeDef *hpll);
float SPLL_GetPhaseDeg(SPLL_HandleTypeDef *hpll);
void SPLL_SetNominalFreq(SPLL_HandleTypeDef *hpll, float freq_hz);

#endif /* __SPLL_H__ */
