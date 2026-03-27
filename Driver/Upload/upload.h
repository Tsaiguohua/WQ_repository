#ifndef UPLOAD_H
#define UPLOAD_H

#include "FreeRTOS.h" // ← 必须在task.h之前
#include "task.h"     // ← 定义TaskHandle_t
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// 数据上传任务 - FreeRTOS版本
//
// 功能说明：
//   1. 周期性将采集的数据打包成JSON格式通过4G模块MQTT上传
//   2. 支持通过MQTT指令动态设置上传频率
//   3. 数据包括：GPS、环境（温湿度、气压、电量）、水质传感器
//
// 硬件配置：
//   - 4G模块: 透传模式，MQTT已配置
//   - 串口: UART4 (PC10/PC11, 115200 baud)
//
// JSON数据格式（参考裸机版本）：
//   {
//     "ID": 3,
//     "Number": "HDY1003",
//     "SN": "0641100302",
//     "Temp": "25.6",
//     "Humi": "60.2",
//     "V": "85%",
//     "Time": "2026.01.04 10:30",
//     "Lat": "30.88497N",
//     "Lon": "121.897293E",
//     "GPS": 1,
//     "Frequency": "60s",
//     "Channel1": 1,
//     "ChannelSensor1": 1,
//     "WTemp": "20.5",
//     "COD": "5.234567",
//     "CDOM": "1.234567",
//     ...
//   }
//
// 创建日期: 2026/01/04
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          上传频率配置
 *===========================================================================*/

#define UPLOAD_FREQ_MIN 10     // 最小上传频率（秒）
#define UPLOAD_FREQ_MAX 3600   // 最大上传频率（秒）= 1小时
#define UPLOAD_FREQ_DEFAULT 10 // 默认上传频率（秒）

#define UPLOAD_BUFFER_SIZE 1200 // 上传缓冲区大小（JSON字符串）

/*============================================================================
 *                          全局变量声明
 *===========================================================================*/

/* 当前上传频率（秒） */
extern uint16_t g_upload_frequency;

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化上传模块
 * @note   在FreeRTOS任务启动前调用
 */
void Upload_Init(void);

/**
 * @brief  初始化上传任务
 * @retval 上传任务句柄
 */
TaskHandle_t Upload_TaskInit(void);

/**
 * @brief  上传任务函数
 * @param  pvParameters: FreeRTOS任务参数（未使用）
 * @note   由Upload_TaskInit内部创建
 */
void Upload_Task(void *pvParameters);

/**
 * @brief  设置上传频率
 * @param  freq_seconds: 上传间隔（秒），范围10~3600
 * @return true:设置成功, false:参数无效
 */
bool Upload_SetFrequency(uint16_t freq_seconds);

/**
 * @brief  获取当前上传频率
 * @return 当前上传间隔（秒）
 */
uint16_t Upload_GetFrequency(void);

/**
 * @brief  立即上传一次数据（不影响定时上传）
 */
void Upload_SendNow(void);

#endif
