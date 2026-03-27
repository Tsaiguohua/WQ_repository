#ifndef GPSTASK_H
#define GPSTASK_H

#include <stdbool.h>
#include "gps.h"
#include "WQInterface.h"

/**
 * @brief  初始化 GPS 任务所需的同步量，由 TasksInit.c 的早期统一调用
 */
void GPS_TaskInit(void);

/**
 * @brief  FreeRTOS GPS 解析任务主入口
 */
void GPS_Task(void *pvParameters);

/**
 * @brief  驱动层的接收中断回调，当 NMEA 帧缓冲就绪时触发
 * @note   供 HardwareInitTask.c 注册给中间层
 */
void GPSTask_RxCallback(void);

/**
 * @brief  获取外部能够读取的最新有效 GPS 数据
 * @param  data 输出缓冲区
 * @return true如果获取成功
 */
bool GPSTask_GetData(gps_data_t *data);

#endif
