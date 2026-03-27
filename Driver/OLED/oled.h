#ifndef OLED_H
#define OLED_H

#include "u8g2.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// OLED显示驱动 - 基于u8g2库
//
// 功能说明：
//   SSD1306 OLED显示屏驱动，128x64分辨率
//   - 开机进度条动画
//   - 主界面显示：时间、电量、温湿度、传感器状态
//
// 硬件配置：
//   - 显示屏: SSD1306 128x64
//   - 接口: I2C软件模拟
//   - SCL: PD0
//   - SDA: PD1
//   - 地址: 0x78
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/* u8g2对象 */
extern u8g2_t u8g2;



/**
 * @brief  初始化OLED（u8g2库）
 */
void OLED_Init(void);

/**
 * @brief  检测按键状态（PB7）
 * @retval 0=按下，1=释放
 */
uint8_t OLED_GetKeyState(void);

/**
 * @brief  显示开机进度条动画
 * @note   进度从0%到100%，带"SHOU-HDY"标题
 */
void OLED_ShowBootAnimation(void);

/**
 * @brief  显示主界面
 * @param  year: 年份字符串（如"2025-12-31"）
 * @param  time: 时间字符串（如"14:30"）
 * @param  battery: 电量百分比（如"95%"）
 * @param  temp: 温度字符串（如"25C"）
 * @param  freq: 频率/其他信息
 * @param  hum: 湿度字符串（如"60%"）
 */
void OLED_ShowMainScreen(const char *year, const char *time, const char *battery, const char *temp, const char *freq, const char *hum, uint8_t ch1, uint8_t ch2, uint8_t ch3);

/**
 * @brief  清屏
 */
void OLED_Clear(void);



#endif
