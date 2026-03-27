#ifndef COMMAND_H
#define COMMAND_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// MQTT指令解析模块 - FreeRTOS版本
//
// 功能说明：
//   解析云平台(MQTT)下发的控制指令，执行相应的操作
//   与裸机版本功能一致，新增采集频率和上传频率分离控制
//
// 【重要】频率控制说明：
//   - 所有频率控制必须通过本模块的JSON格式指令统一管理
//   - acquisition 和 upload 模块仅提供 SetFrequency/GetFrequency API
//   - 禁止直接在 acquisition 和 upload 模块中解析命令
//
// 支持的指令格式（严格JSON格式）：
//   1. 频率控制：
//      - {"Frequency":XX}          设置采集和上传频率（向后兼容）
//      - {"AcqFrequency":XX}       单独设置采集频率
//      - {"UploadFrequency":XX}    单独设置上传频率
//      说明：XX为数值，如 {"Frequency":60} 表示60秒
//
//   2. 数据获取：
//      - {"GET_MMSG":1}            获取设备基本信息
//      - {"GET_DMSG":1}            获取水质传感器数据
//
//   3. 通道控制：
//      - {"ONECHANNEL_OPEN":1}     打开通道1
//      - {"ONECHANNEL_CLOSE":1}    关闭通道1
//      - {"TWOCHANNEL_OPEN":1}     打开通道2
//      - {"TWOCHANNEL_CLOSE":1}    关闭通道2
//      - {"THREECHANNEL_OPEN":1}   打开通道3
//      - {"THREECHANNEL_CLOSE":1}  关闭通道3
//      - {"ALL_CHANNEL_OPEN":1}    打开所有通道
//      - {"ALL_CHANNEL_CLOSE":1}   关闭所有通道
//
//   4. 系统控制：
//      - {"RESTART":1}             重启系统
//      - {"BUZZER_OPEN":1}         打开蜂鸣器
//      - {"BUZZER_CLOSE":1}        关闭蜂鸣器
//
// 创建日期: 2026/01/04
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化指令解析模块
 * @note   在系统初始化时调用
 */
void Command_Init(void);

/**
 * @brief  解析并执行MQTT指令
 * @param  cmd_str: 指令字符串(JSON格式或纯文本或二进制OTA数据)
 * @param  len: 数据实际长度（用于支持二进制数据，不能用strlen）
 * @return true:指令执行成功, false:指令格式错误或执行失败
 * @note   在UART接收到完整数据包后调用
 */
bool Command_Parse(const char *cmd_str, uint16_t len);

/**
 * @brief  获取最后一次指令执行状态
 * @return 指令执行状态字符串
 */
const char *Command_GetLastStatus(void);

/**
 * @brief  Command任务函数(FreeRTOS任务)
 * @param  pvParameters: 任务参数(未使用)
 * @note   阻塞等待信号量,接收到MQTT指令后解析执行
 *         在app.c中创建此任务
 */
void Command_Task(void *pvParameters);

/*============================================================================
 *                          外部变量声明
 *===========================================================================*/

/* Command信号量句柄(在uart4_mqtt.c中使用) */
extern SemaphoreHandle_t xCommandSemaphore;

#endif
