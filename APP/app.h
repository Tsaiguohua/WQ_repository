#ifndef __APP_H__
#define __APP_H__
#include "FreeRTOS.h"
#include "task.h"

//////////////////////////////////////////////////////////////////////////////////
// APP应用层 - 任务管理中心
//
// 功能说明：
//   负责创建和管理所有业务任务
//
// 创建日期: 2026/01/04
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  APP初始化任务（启动任务）
 * @note   负责创建所有业务任务，完成后自删除
 */
void app_init_task(void *pvParameters);

/**
 * @brief  OLED控制任务
 * @note   检测按键状态，控制OLED显示
 */
void app_oled_task(void *pvParameters);

/**
 * @brief  APP业务任务
 * @note   处理环境数据的定期采集（AHT20温湿度 + 电量）
 */
void app_task(void *pvParameters);

#endif
