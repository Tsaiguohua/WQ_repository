#ifndef VOLTAGE_H
#define VOLTAGE_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// 电压/电量检测驱动
//
// 功能说明：
//   通过ADC采样电压检测模块的输出，计算电池电量百分比
//   12V 14400mAh锂电池供电
//
// 硬件配置：
//   - ADC引脚: PA1 (ADC1_Channel_1)
//   - 采样方式: 软件触发，单次转换
//
// 电量计算：
//   使用线性映射公式：
//   电量% = (ADC值 - ADC_MIN) / (ADC_MAX - ADC_MIN) * 100
//   - ADC_MAX (满电): 1301
//   - ADC_MIN (空电): 908
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/* 电量数据（全局变量） */
extern uint8_t battery;          // 电量百分比（0-100%）
extern float voltage_percentage; // 电量百分比（浮点数）

/**
 * @brief  初始化电压检测（ADC）
 */
void Voltage_Init(void);

/**
 * @brief  读取电量百分比
 * @return 电量百分比（0-100）
 * @note   内部做了40次采样平均，防止波动
 */
uint8_t Voltage_GetBattery(void);

/**
 * @brief  读取原始ADC值
 * @return ADC值（0-4095）
 */
uint16_t Voltage_GetADC(void);

/**
 * @brief  检查是否低电量
 * @return true:低电量（<10%）, false:正常
 */
bool Voltage_IsLow(void);

#endif
