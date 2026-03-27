#ifndef OLEDTASK_H
#define OLEDTASK_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  OLED 刷新任务
 * @note   从各种底层模块收集数据，传递给 WQInterface.Display 进行更新
 */
void OledTask(void *pvParameters);

#endif
