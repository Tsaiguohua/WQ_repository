#include "watchdog.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "task.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// 独立看门狗（IWDG）驱动实现 - FreeRTOS版本
//
// 功能说明：
//   1. STM32F407使用独立看门狗（IWDG），基于内部LSI时钟（32kHz）
//   2. 超时计算公式：Timeout = (Prescaler / LSI_Freq) * Reload
//      - LSI频率：32kHz
//      - 分频系数：256
//      - 重装值：3750
//      - 超时时间：(256 / 32000) * 3750 = 30秒
//
// 创建日期: 2026/01/05
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          硬件配置常量
 *===========================================================================*/

// STM32F407 IWDG配置
#define LSI_FREQUENCY 32000    // LSI时钟频率（Hz）
#define IWDG_PRESCALER_VAL 256 // 分频系数
#define IWDG_RELOAD_VAL 3750   // 重装载值（30秒超时）

// 超时时间验证：(256 / 32000) * 3750 = 30秒

/*============================================================================
 *                          硬件初始化
 *===========================================================================*/

/**
 * @brief  初始化独立看门狗硬件
 * @note   配置IWDG，超时时间30秒
 */
void Watchdog_Init(void) {
  /* 使能IWDG寄存器写访问 */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* 设置分频系数：256 */
  IWDG_SetPrescaler(IWDG_Prescaler_256);

  /* 设置重装载值：3750（30秒超时） */
  IWDG_SetReload(IWDG_RELOAD_VAL);

  /* 重装载计数器（喂狗） */
  IWDG_ReloadCounter();

  /* 启动IWDG */
  IWDG_Enable();

  printf("[Watchdog] Hardware initialized: timeout=%ds\r\n",
         WATCHDOG_TIMEOUT_SEC);
}

/*============================================================================
 *                          看门狗任务
 *===========================================================================*/

/**
 * @brief  创建看门狗任务
 * @retval 看门狗任务句柄
 */
TaskHandle_t Watchdog_TaskInit(void) {
  BaseType_t result;
  TaskHandle_t watchdog_task_handle = NULL; // ← 保存句柄

  /* 创建看门狗任务（最低优先级1） */
  result = xTaskCreate(Watchdog_Task,        // 任务函数
                       "watchdog",           // 任务名称
                       192,                  // 堆栈192字(768B)，喂狗+printf
                       NULL,                 // 任务参数
                       1,                    // 优先级（最低）
                       &watchdog_task_handle // ← 传出句柄
  );

  if (result != pdPASS) {
    printf("[Watchdog] Task creation failed!\r\n");
    while (1)
      ; // 创建失败，停止运行
  }

  printf("[Watchdog] Task created, priority=1, feed interval=%dms\r\n",
         WATCHDOG_FEED_INTERVAL);

  return watchdog_task_handle; // ← 返回句柄
}

/**
 * @brief  看门狗任务函数
 * @note   最低优先级任务，每15秒喂一次狗
 */
void Watchdog_Task(void *pvParameters) {
  TickType_t last_wake_time;
  const TickType_t feed_interval = pdMS_TO_TICKS(WATCHDOG_FEED_INTERVAL);
  uint32_t feed_count = 0;

  /* 获取当前时间 */
  last_wake_time = xTaskGetTickCount();

  printf("[Watchdog] Task started, feeding every %ds\r\n",
         WATCHDOG_FEED_INTERVAL / 1000);

  while (1) {
    /* 喂狗 */
    IWDG_ReloadCounter();
    feed_count++;

    /* 每10次打印一次日志（每150秒） */
    if (feed_count % 10 == 0) {
      printf("[Watchdog] Fed (count=%lu)\r\n", feed_count);
    }

    /* 周期性延时（15秒） */
    vTaskDelayUntil(&last_wake_time, feed_interval);
  }
}

/*============================================================================
 *                          辅助函数
 *===========================================================================*/

/**
 * @brief  手动喂狗（仅供调试使用）
 */
void Watchdog_Feed(void) {
  IWDG_ReloadCounter();
  printf("[Watchdog] Manual feed\r\n");
}
