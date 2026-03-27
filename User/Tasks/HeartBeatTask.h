#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include "FreeRTOS.h"
#include "task.h"

/* 心跳包相关定义 */
#define HEARTBEAT_INTERVAL 240 // 心跳间隔，单位秒
#define HEARTBEAT_BUFFER_SIZE 256

/**
 * @brief  初始化心跳模块底层（可选）
 */
void Heartbeat_Init(void);

/**
 * @brief  FreeRTOS 心跳任务的主入口
 * @note   被系统级的 TasksInit.c 所创建和调度
 */
void Heartbeat_Task(void *pvParameters);

#endif
