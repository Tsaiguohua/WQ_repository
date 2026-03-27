#ifndef TASKSINIT_H
#define TASKSINIT_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//////////////////////////////////////////////////////////////////////////////////
// 任务统一初始化模块
//
// 功能说明：
//   本模块是 FreeRTOS 任务的统一"注册中心"。
//   负责创建所有业务任务（采集、上传、OLED、命令、OTA等）
//   以及创建所有任务间通信所需的队列和信号量。
//
//   对应 OV-Watch 项目中的 user_TasksInit.c
//
// 启动流程：
//   main() → xTaskCreate(HardwareInitTask) → vTaskStartScheduler()
//                ↓ HardwareInitTask 完成硬件初始化
//                ↓ vTaskResumeAll() 后各任务按优先级运行
//
// 注意事项：
//   ⚠️ User_Tasks_Init() 在 main() 中调用（调度器启动前）
//   ⚠️ HardwareInitTask 优先级最高，确保硬件最先初始化
//   ⚠️ HardwareInitTask 完成后自删除
//
// 作者：蔡国华
// 创建日期：2026/03/18
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          外部共享句柄（其他模块可引用）
 *===========================================================================*/

/* 任务句柄（供 OTA 暂停/恢复任务使用） */
extern TaskHandle_t AcqTaskHandle;
extern TaskHandle_t UploadTaskHandle;
extern TaskHandle_t OledTaskHandle;
extern TaskHandle_t CmdTaskHandle;

/* 队列句柄（外部可引用） */
extern QueueHandle_t xUartRxQueue;

/*============================================================================
 *                          API
 *===========================================================================*/

/**
 * @brief  统一任务初始化函数
 * @note   在 main() 中、vTaskStartScheduler() 之前调用
 *         创建所有业务任务和通信对象
 */
void User_Tasks_Init(void);

#endif
