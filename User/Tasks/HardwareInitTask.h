#ifndef HARDWAREINITTASK_H
#define HARDWAREINITTASK_H

#include "FreeRTOS.h"
#include "task.h"

//////////////////////////////////////////////////////////////////////////////////
// 硬件初始化任务模块
//
// 功能说明：
//   上电后第一个运行的任务（优先级最高）。
//   负责按顺序初始化所有硬件和传感器，完成后自删除。
//
//   对应 OV-Watch 项目中的 user_HardwareInitTask.c
//
// 初始化顺序：
//   1. 挂起调度器（防止业务任务在初始化中途抢占）
//   2. RS485 总线初始化
//   3. OLED 初始化（必须在任务中，U8g2 堆栈消耗大）
//   4. 电量检测初始化（WQInterface.Battery.Init）
//   5. AHT20 温湿度传感器初始化（带重试）
//   6. 水质传感器自检（SelfExam_StartExaminate）
//   7. 通道绑定（WQ_Interface_Bind）
//   8. 恢复调度器
//   9. 自删除
//
// 注意事项：
//   ⚠️ OLED_Init() 必须在此任务中调用，不能在 main() 中
//   ⚠️ SelfExam_StartExaminate() 内部使用 vTaskDelay，不能在禁止调度期间调用
//   ⚠️ 本任务完成后自我删除，释放内存
//
// 作者：蔡国华
// 创建日期：2026/03/18
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  硬件初始化任务函数
 * @param  argument: 未使用
 * @note   由 User_Tasks_Init() 以最高优先级创建
 */
void HardwareInitTask(void *argument);

#endif
