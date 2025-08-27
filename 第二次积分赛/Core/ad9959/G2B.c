#include "G2B.h"
#include "interact.h"
// 频率-电压补偿表（根据手册测试数据）
static const FreqVppCompensation vpp_comp_table[] = 
{
 
    {0, 500.0f}            // 默认值
};

// 获取当前频率下的最大输出电压
static float get_max_vpp_for_freq(uint32_t freq_hz) {
    for(int i = 0; i < sizeof(vpp_comp_table)/sizeof(vpp_comp_table[0]); i++) {
        if(freq_hz <= vpp_comp_table[i].freq_hz) {
            return vpp_comp_table[i].max_vpp;
        }
    }
    return 500.0f; // 默认值
}

// 设置输出电压峰峰值
void G2B_Set_Vpp(uint8_t Channel, uint32_t freq_hz, uint16_t Vpp_mV) {
    // 电压范围限制
    if(Vpp_mV < VPP_MIN) Vpp_mV = VPP_MIN;
    if(Vpp_mV > VPP_MAX) Vpp_mV = VPP_MAX;
    
    // 获取频率补偿后的最大输出电压
    float max_vpp = get_max_vpp_for_freq(freq_hz);
    
    // 计算幅度值 (0-1023)
    uint16_t amplitude = (uint16_t)((float)Vpp_mV * MAX_AMPLITUDE / max_vpp);
    if(amplitude > MAX_AMPLITUDE) amplitude = MAX_AMPLITUDE;
    
    // 使用AD9959原有函数设置幅度
    AD9959_Set_Amp(Channel, amplitude);
}
//////////////////////////////////////////////
/*计算峰峰值*/
float CalculateVpp(uint16_t *adc_buffer, uint16_t length)
{
    if (length == 0) return 0.0f;
    
    uint16_t min = adc_buffer[0];
    uint16_t max = adc_buffer[0];
    
    // 查找最小值和最大值
    for (uint16_t i = 1; i < length; i++) {
        if (adc_buffer[i] < min) min = adc_buffer[i];
        if (adc_buffer[i] > max) max = adc_buffer[i];
    }
    
    // 计算峰峰值（数字值）并转换为电压
    uint16_t vpp_digital = max - min;
    float vpp_voltage = vpp_digital * 3.3f / 4095.0f;
    
    return vpp_voltage;
}
////////////////////////////////////////////////////

