#ifndef __PLL_H__
#define __PLL_H__

#include <stdint.h>

// ���໷�������ýṹ��
typedef struct {
    float k;          // SOGI����ϵ�� (�Ƽ���2)
    float omega_n;    // ���Ƶ��(rad/s)
    float kp;         // ����ϵ��
    float ki;         // ����ϵ��
} SPLL_ConfigTypeDef;

// ���໷״̬�ṹ��
typedef struct {
    // ���ò���
    SPLL_ConfigTypeDef config;
    
    // ״̬����
    float v[2];       // SOGI��ͨ��״̬
    float qv[2];      // SOGI����ͨ��״̬
    float omega;      // ���ƽ�Ƶ��(rad/s)
    float theta;      // ������λ(rad)
    float int_err;    // �������
    uint32_t counter; // ����������
    
    // �м��������
    float _v_dq_d;    // d�����
    float _v_dq_q;    // q�����
} SPLL_HandleTypeDef;

// ����ԭ��
void SPLL_Init(SPLL_HandleTypeDef *hpll, const SPLL_ConfigTypeDef *config);
void SPLL_Update(SPLL_HandleTypeDef *hpll, float vin, float Ts);
void SPLL_Reset(SPLL_HandleTypeDef *hpll);
float SPLL_GetFrequency(SPLL_HandleTypeDef *hpll);
float SPLL_GetPhaseRad(SPLL_HandleTypeDef *hpll);
float SPLL_GetPhaseDeg(SPLL_HandleTypeDef *hpll);
void SPLL_SetNominalFreq(SPLL_HandleTypeDef *hpll, float freq_hz);

#endif /* __SPLL_H__ */
