#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// 蜂鸣器驱动 - 面向对象风格
//
// 功能说明：
//   控制蜂鸣器发出不同的提示音
//   - 短响：识别到传感器时
//   - 长响：报警或错误
//   - 开机音：系统启动完成
//
// 硬件配置：
//   - 蜂鸣器引脚: PB7
//   - 控制方式: 高电平响，低电平停
//
// 目标平台: STM32F407ZGT6
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  初始化蜂鸣器
 * @note   配置GPIO为推挽输出
 */
void Buzzer_Init(void);

/**
 * @brief  蜂鸣器开启（持续响）
 */
void Buzzer_On(void);

/**
 * @brief  蜂鸣器关闭
 */
void Buzzer_Off(void);

/**
 * @brief  蜂鸣器短响一下（100ms）
 * @note   用于传感器识别成功提示
 */
void Buzzer_Beep(void);

/**
 * @brief  蜂鸣器响指定时间
 * @param  ms: 响的时间（毫秒）
 */
void Buzzer_BeepMs(uint32_t ms);

/**
 * @brief  开机提示音
 * @note   长响1秒 + 短响2下
 */
void Buzzer_Startup(void);

/**
 * @brief  错误/报警提示音
 * @note   连续响3下
 */
void Buzzer_Alarm(void);

#endif
