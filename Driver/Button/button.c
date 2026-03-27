#include "button.h"
#include "FreeRTOS.h"
#include "task.h"

//////////////////////////////////////////////////////////////////////////////////
// 按键驱动实现 - FreeRTOS版本
//
// 功能说明：
//   提供按键检测功能，控制OLED显示开关
//
// 创建日期: 2026/01/03
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          按键初始化
 *===========================================================================*/

/**
 * @brief  初始化按键GPIO
 */
void Button_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* 使能GPIO时钟 */
  RCC_AHB1PeriphClockCmd(BUTTON_GPIO_CLK, ENABLE);

  /* 配置按键引脚（PC0）为上拉输入 */
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
  GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
}

/*============================================================================
 *                          按键检测函数
 *===========================================================================*/

/**
 * @brief  读取按键状态
 * @retval 0=按下，1=释放
 */
uint8_t Button_GetState(void) {
  return GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN);
}

/**
 * @brief  扫描按键（带消抖）
 * @retval 1=检测到按键按下事件，0=无事件
 * @note   使用FreeRTOS延时进行消抖
 */
uint8_t Button_Scan(void) {
  static uint8_t last_state = 1; // 上次按键状态（1=释放）
  uint8_t current_state;

  current_state = Button_GetState();

  /* 检测下降沿（按键按下） */
  if (last_state == 1 && current_state == 0) {
    vTaskDelay(pdMS_TO_TICKS(20)); // 消抖延时20ms

    /* 等待按键释放 */
    while (Button_GetState() == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 释放后消抖
    last_state = 1;
    return 1; // 返回按键按下事件
  }

  last_state = current_state;
  return 0; // 无按键事件
}
