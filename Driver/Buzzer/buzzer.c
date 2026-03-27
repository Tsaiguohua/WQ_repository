#include "buzzer.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"


//////////////////////////////////////////////////////////////////////////////////
// 蜂鸣器驱动实现
//
// 硬件配置：
//   - 蜂鸣器引脚: PB7（从JP2排针引出）
//   - 控制方式: 高电平响，低电平停（有源蜂鸣器）
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          硬件配置宏定义
 *===========================================================================*/

#define BUZZER_GPIO_PORT GPIOB     // 蜂鸣器端口
#define BUZZER_GPIO_PIN GPIO_Pin_7 // 蜂鸣器引脚 PB7
#define BUZZER_GPIO_CLK RCC_AHB1Periph_GPIOB

/* 蜂鸣器控制宏 */
#define BUZZER_ON() GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)
#define BUZZER_OFF() GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)

/*============================================================================
 *                          API实现
 *===========================================================================*/

/**
 * @brief  初始化蜂鸣器GPIO
 */
void Buzzer_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* 使能GPIO时钟 */
  RCC_AHB1PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);

  /* 配置蜂鸣器引脚为推挽输出 */
  GPIO_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  // 输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // 下拉（默认不响）
  GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);

  /* 初始状态：关闭蜂鸣器 */
  BUZZER_OFF();
}

/**
 * @brief  蜂鸣器开启
 */
void Buzzer_On(void) { BUZZER_ON(); }

/**
 * @brief  蜂鸣器关闭
 */
void Buzzer_Off(void) { BUZZER_OFF(); }

/**
 * @brief  蜂鸣器短响一下（200ms）
 */
void Buzzer_Beep(void) {
  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(200));
  BUZZER_OFF();
}

/**
 * @brief  蜂鸣器响指定时间
 */
void Buzzer_BeepMs(uint32_t ms) {
  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(ms));
  BUZZER_OFF();
}

/**
 * @brief  开机提示音
 * @note   长响400ms
 */
void Buzzer_Startup(void) {
  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(400));
  BUZZER_OFF();
}

/**
 * @brief  错误/报警提示音
 * @note   长响1秒，然后短响2下
 */
void Buzzer_Alarm(void) {
  /* 长响1秒 */
  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(1000));
  BUZZER_OFF();
  vTaskDelay(pdMS_TO_TICKS(100));

  /* 短响2下 */
  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(100));
  BUZZER_OFF();
  vTaskDelay(pdMS_TO_TICKS(100));

  BUZZER_ON();
  vTaskDelay(pdMS_TO_TICKS(100));
  BUZZER_OFF();
}
