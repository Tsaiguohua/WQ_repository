#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "FreeRTOS.h" // ← 必须在task.h之前
#include "task.h"     // ← 定义TaskHandle_t
#include <stdbool.h>
#include <stdint.h>


//////////////////////////////////////////////////////////////////////////////////
// 独立看门狗（IWDG）驱动 - FreeRTOS版本
//
// 功能说明：
//   1. 使用STM32的独立看门狗（IWDG）监控系统运行状态
//   2. 创建专用看门狗任务，定期喂狗（15秒一次）
//   3. 超时时间设置为30秒，防止系统死锁或程序跑飞
//
// 工作原理：
//   - 硬件初始化时启动IWDG，超时时间30秒
//   - 看门狗任务每15秒喂一次狗
//   - 如果系统死锁/卡死，看门狗任务无法运行，30秒后自动复位
//
// 使用方法：
//   1. 在main()中调用 Watchdog_Init() 初始化硬件
//   2. 在app_init_task()中调用 Watchdog_TaskInit() 创建任务
//
// 创建日期: 2026/01/05
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          配置参数
 *===========================================================================*/

#define WATCHDOG_TIMEOUT_SEC 30      // 看门狗超时时间（秒）
#define WATCHDOG_FEED_INTERVAL 15000 // 喂狗间隔（毫秒）

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化独立看门狗硬件
 * @note   配置IWDG，超时时间30秒
 *         必须在FreeRTOS调度器启动前调用
 */
void Watchdog_Init(void);

/**
 * @brief  创建看门狗任务
 * @retval 看门狗任务句柄
 * @note   在FreeRTOS调度器启动后调用
 *         任务优先级为1（最低），每15秒喂一次狗
 */
TaskHandle_t Watchdog_TaskInit(void);

/**
 * @brief  看门狗任务函数
 * @param  pvParameters: FreeRTOS任务参数（未使用）
 * @note   由xTaskCreate创建，不需要手动调用
 */
void Watchdog_Task(void *pvParameters);

/**
 * @brief  手动喂狗（仅供调试使用）
 * @note   正常情况下由看门狗任务自动喂狗，无需手动调用
 */
void Watchdog_Feed(void);

#endif // WATCHDOG_H
