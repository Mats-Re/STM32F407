#include "PLL.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define TWOPI (2.0f*PI)

// 初始化锁相环
void SPLL_Init(SPLL_HandleTypeDef *hpll, const SPLL_ConfigTypeDef *config) {
    // 拷贝配置参数
    hpll->config = *config;
    
    // 初始化状态变量
    hpll->v[0] = hpll->v[1] = 0.0f;
    hpll->qv[0] = hpll->qv[1] = 0.0f;
    hpll->omega = hpll->config.omega_n;
    hpll->theta = 0.0f;
    hpll->int_err = 0.0f;
    hpll->counter = 0;
    hpll->_v_dq_d = 0.0f;
    hpll->_v_dq_q = 0.0f;
}

// 复位锁相环状态
void SPLL_Reset(SPLL_HandleTypeDef *hpll) {
    hpll->v[0] = hpll->v[1] = 0.0f;
    hpll->qv[0] = hpll->qv[1] = 0.0f;
    hpll->omega = hpll->config.omega_n;
    hpll->theta = 0.0f;
    hpll->int_err = 0.0f;
    hpll->counter = 0;
}

// 更新锁相环状态
void SPLL_Update(SPLL_HandleTypeDef *hpll, float vin, float Ts) {
    // SOGI正交信号生成
    float err = vin - hpll->v[1];
    
    // 更新状态寄存器
    hpll->v[0] = hpll->v[1];
    hpll->qv[0] = hpll->qv[1];
    
    // 计算新状态
    hpll->v[1] = err * hpll->config.k * hpll->omega * Ts 
               + hpll->v[0] 
               - hpll->qv[0] * hpll->omega * Ts;
    
    hpll->qv[1] = hpll->v[0] * hpll->omega * Ts 
                + hpll->qv[0];
    
    // Park变换
    float sin_theta = sinf(hpll->theta);
    float cos_theta = cosf(hpll->theta);
    
    hpll->_v_dq_d = hpll->v[1] * cos_theta + hpll->qv[1] * sin_theta;
    hpll->_v_dq_q = hpll->qv[1] * cos_theta - hpll->v[1] * sin_theta;
    
    // PI控制器
    float omega_err = hpll->_v_dq_q;
    hpll->int_err += omega_err * hpll->config.ki * Ts;
    hpll->omega = hpll->config.omega_n + omega_err * hpll->config.kp + hpll->int_err;
    
    // 更新相位(归一化到0-2π)
    hpll->theta += hpll->omega * Ts;
    if(hpll->theta > TWOPI) hpll->theta -= TWOPI;
    if(hpll->theta < 0.0f) hpll->theta += TWOPI;
    
    // 更新计数器
    hpll->counter++;
}

// 获取估计频率(Hz)
float SPLL_GetFrequency(SPLL_HandleTypeDef *hpll) {
    return hpll->omega / TWOPI;
}

// 获取估计相位(弧度)
float SPLL_GetPhaseRad(SPLL_HandleTypeDef *hpll) {
    return hpll->theta;
}

// 获取估计相位(度数)
float SPLL_GetPhaseDeg(SPLL_HandleTypeDef *hpll) {
    return hpll->theta * 180.0f / PI;
}

// 设置额定频率(Hz)
void SPLL_SetNominalFreq(SPLL_HandleTypeDef *hpll, float freq_hz) {
    hpll->config.omega_n = TWOPI * freq_hz;
}
