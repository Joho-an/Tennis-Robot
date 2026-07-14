#ifndef __BATTERY_H
#define __BATTERY_H

#include "main.h"

/**
 * @brief 初始化电量监测模块
 * @param hadc: 传入 ADC 句柄 (如 &hadc1)
 */
void Battery_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief 采集电池电压并返回物理值
 * @return 电池实际电压 (mV)
 */
uint16_t Battery_Read_Voltage(void);

/**
 * @brief 锂电池电压转百分比 (3S 锂电池专用曲线)
 */
uint8_t Battery_Calculate_Percentage(uint16_t v_in);

/**
 * @brief 在 LCD 上实时刷新电量信息
 */
void Battery_Show(void);

#endif /* __BATTERY_H */