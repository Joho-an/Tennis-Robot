#include "battery.h"
#include <stdio.h>
#include "LCD.h" 

static ADC_HandleTypeDef *bat_hadc;

/**
 * @brief 初始化电量监测模块
 * @param hadc: 传入 ADC1 的句柄
 */
void Battery_Init(ADC_HandleTypeDef *hadc) {
    bat_hadc = hadc;
}

/**
 * @brief 采集电池电压
 * 硬件引脚：PA6 (对应 ADC1_IN3)
 */
uint16_t Battery_Read_Voltage(void) {
    uint32_t adc_accumulated = 0;
    const uint8_t samples = 30; 

    for (uint8_t i = 0; i < samples; i++) {
        // 启动 ADC1
        HAL_ADC_Start(bat_hadc);
        
        // H750 转换速度极快，1ms 超时
        if (HAL_ADC_PollForConversion(bat_hadc, 1) == HAL_OK) {
            adc_accumulated += HAL_ADC_GetValue(bat_hadc);
        }
        HAL_ADC_Stop(bat_hadc); 
        
        // 增加软件延时，分散采样点以避开同频的高频噪声
        for(volatile int d = 0; d < 2000; d++);
    }

    /* --- H750 核心配置确认 --- */
    // --16bit-65535--、--12bit 4095--
    uint16_t v_pin_mv = (uint32_t)(adc_accumulated / samples) * 3300 / 65535;
    
    // 保持分压比例 10:1 (V_bat = V_pin * 11)
    return v_pin_mv * 11; 
}

/**
 * @brief 锂电池电压转百分比 (3S 锂电池专用曲线)
 */
uint8_t Battery_Calculate_Percentage(uint16_t v_in) {
    if (v_in >= 12600) return 100;
    if (v_in <= 9600)  return 0;

    if (v_in > 11500) return (uint8_t)((v_in - 11500) * 50 / (12600 - 11500) + 50); 
    if (v_in > 10500) return (uint8_t)((v_in - 10500) * 40 / (11500 - 10500) + 10); 
    return (uint8_t)((v_in - 9600) * 10 / (10500 - 9600));                                                            
}

/**
 * @brief  LCD 显示电池电压和百分比
 */
void Battery_Show(void) {
    uint16_t vol = Battery_Read_Voltage();
    uint8_t Bat = Battery_Calculate_Percentage(vol);
    
    uint16_t text_color = (Bat <= 10) ? RED : GREEN; 
    uint16_t bg_color = BLACK; 
    
    char buf_v[24], buf_p[24];
    
    sprintf(buf_v, "Vol:%d.%02dV  ", vol / 1000, (vol % 1000) / 10);
    sprintf(buf_p, "Bat:%d%%    ", Bat); 

    // 显示位置
    ST7735_DrawString(90, 5, buf_v, text_color, bg_color);
    ST7735_DrawString(90, 15, buf_p, text_color, bg_color);
}