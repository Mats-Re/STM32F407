#include "PLL.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define TWOPI (2.0f*PI)

// ��ʼ�����໷
void SPLL_Init(SPLL_HandleTypeDef *hpll, const SPLL_ConfigTypeDef *config) {
    // �������ò���
    hpll->config = *config;
    
    // ��ʼ��״̬����
    hpll->v[0] = hpll->v[1] = 0.0f;
    hpll->qv[0] = hpll->qv[1] = 0.0f;
    hpll->omega = hpll->config.omega_n;
    hpll->theta = 0.0f;
    hpll->int_err = 0.0f;
    hpll->counter = 0;
    hpll->_v_dq_d = 0.0f;
    hpll->_v_dq_q = 0.0f;
}

// ��λ���໷״̬
void SPLL_Reset(SPLL_HandleTypeDef *hpll) {
    hpll->v[0] = hpll->v[1] = 0.0f;
    hpll->qv[0] = hpll->qv[1] = 0.0f;
    hpll->omega = hpll->config.omega_n;
    hpll->theta = 0.0f;
    hpll->int_err = 0.0f;
    hpll->counter = 0;
}

// �������໷״̬
void SPLL_Update(SPLL_HandleTypeDef *hpll, float vin, float Ts) {
    // SOGI�����ź�����
    float err = vin - hpll->v[1];
    
    // ����״̬�Ĵ���
    hpll->v[0] = hpll->v[1];
    hpll->qv[0] = hpll->qv[1];
    
    // ������״̬
    hpll->v[1] = err * hpll->config.k * hpll->omega * Ts 
               + hpll->v[0] 
               - hpll->qv[0] * hpll->omega * Ts;
    
    hpll->qv[1] = hpll->v[0] * hpll->omega * Ts 
                + hpll->qv[0];
    
    // Park�任
    float sin_theta = sinf(hpll->theta);
    float cos_theta = cosf(hpll->theta);
    
    hpll->_v_dq_d = hpll->v[1] * cos_theta + hpll->qv[1] * sin_theta;
    hpll->_v_dq_q = hpll->qv[1] * cos_theta - hpll->v[1] * sin_theta;
    
    // PI������
    float omega_err = hpll->_v_dq_q;
    hpll->int_err += omega_err * hpll->config.ki * Ts;
    hpll->omega = hpll->config.omega_n + omega_err * hpll->config.kp + hpll->int_err;
    
    // ������λ(��һ����0-2��)
    hpll->theta += hpll->omega * Ts;
    if(hpll->theta > TWOPI) hpll->theta -= TWOPI;
    if(hpll->theta < 0.0f) hpll->theta += TWOPI;
    
    // ���¼�����
    hpll->counter++;
}

// ��ȡ����Ƶ��(Hz)
float SPLL_GetFrequency(SPLL_HandleTypeDef *hpll) {
    return hpll->omega / TWOPI;
}

// ��ȡ������λ(����)
float SPLL_GetPhaseRad(SPLL_HandleTypeDef *hpll) {
    return hpll->theta;
}

// ��ȡ������λ(����)
float SPLL_GetPhaseDeg(SPLL_HandleTypeDef *hpll) {
    return hpll->theta * 180.0f / PI;
}

// ���öƵ��(Hz)
void SPLL_SetNominalFreq(SPLL_HandleTypeDef *hpll, float freq_hz) {
    hpll->config.omega_n = TWOPI * freq_hz;
}
