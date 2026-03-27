#ifndef __TFTASK_H
#define __TFTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  获取 TF 卡记录频率（秒）
 */
uint16_t TFTask_GetFrequency(void);

/**
 * @brief  设置 TF 卡记录频率（秒）
 */
bool TFTask_SetFrequency(uint16_t freq_seconds);

/**
 * @brief  TF 卡数据记录任务实体
 */
void TF_Task(void *pvParameters);

#endif
