#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include "FreeRTOS.h"
#include "task.h" // ← 添加task.h定义TaskHandle_t>
#include <stdbool.h>
#include <stdint.h>


//////////////////////////////////////////////////////////////////////////////////
// 设备心跳任务 - FreeRTOS版本
//
// 功能说明：
//   1. 周期性发送设备在线状态心跳消息到MQTT服务器
//   2. 固定240秒周期，独立于数据上传频率
//   3. 轻量级JSON格式，减少流量消耗
//
// 硬件配置：
//   - 4G模块: 透传模式，MQTT已配置
//   - 串口: UART4 (PC10/PC11, 115200 baud)
//
// JSON心跳格式：
//   {
//     "MessageType": "Heartbeat",
//     "DeviceID": "WaterMonitor_001",
//     "Time": "2026.01.27 20:04",
//     "Status": "Online",
//     "Battery": 95,
//     "Frequency": 60
//   }
//
// 注意事项：
//   ⚠️ 与Upload_Task共享UART4，需通过互斥锁保护
//   ⚠️ 心跳周期固定240秒，不可通过MQTT指令修改
//
// 创建日期: 2026/01/27
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          心跳配置
 *===========================================================================*/

#define HEARTBEAT_INTERVAL 240    // 心跳间隔（秒）= 4分钟
#define HEARTBEAT_BUFFER_SIZE 300 // 心跳JSON缓冲区大小

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化心跳模块
 * @note   在FreeRTOS任务启动前调用
 */
void Heartbeat_Init(void);

/**
 * @brief  初始化心跳任务
 * @note   在FreeRTOS调度器启动后调用（app_init_task中）
 * @retval 心跳任务句柄
 */
TaskHandle_t Heartbeat_TaskInit(void);

/**
 * @brief  心跳任务函数
 * @param  pvParameters: FreeRTOS任务参数（未使用）
 * @note   由Heartbeat_TaskInit内部创建
 */
void Heartbeat_Task(void *pvParameters);

#endif
