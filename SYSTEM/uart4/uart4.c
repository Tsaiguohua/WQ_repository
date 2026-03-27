#include "uart4.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "task.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// UART4 MQTT指令接收驱动实现
//
// 功能说明:
//   基于STM32F407的UART4实现MQTT指令接收
//   使用FreeRTOS信号量通知Command任务处理
//
// 硬件配置:
//   - UART: UART4
//   - TX引脚: PC10
//   - RX引脚: PC11
//   - 波特率: 115200
//   - 数据位: 8位
//   - 停止位: 1位
//   - 校验位: 无
//
// 中断处理:
//   使用RXNE中断接收数据,检测JSON边界({...})
//
// 注意事项:
//   1. 中断优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
//   2. 信号量句柄需要在外部(command.c)定义
//
// 创建日期: 2026/01/06
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          硬件配置宏定义
 *===========================================================================*/

#define UART4_BAUDRATE 115200

/* GPIO配置 */
#define UART4_GPIO_CLK RCC_AHB1Periph_GPIOC
#define UART4_TX_PORT GPIOC
#define UART4_TX_PIN GPIO_Pin_10
#define UART4_TX_SOURCE GPIO_PinSource10
#define UART4_RX_PORT GPIOC
#define UART4_RX_PIN GPIO_Pin_11
#define UART4_RX_SOURCE GPIO_PinSource11
#define UART4_AF GPIO_AF_UART4

/* UART配置 */
#define UART4_CLK RCC_APB1Periph_UART4
#define UART4_IRQn UART4_IRQn

/*============================================================================
 *                          模块私有变量
 *===========================================================================*/

/* 双缓冲区定义 (Ping-Pong机制) */
static uint8_t mqtt_rx_buffer[2][MQTT_RX_BUF_SIZE];

/* 当前DMA正在使用的缓冲区索引 (0 或 1) */
static volatile uint8_t g_rx_buf_index = 0;

/* UART接收消息队列句柄（在command.c中定义和创建）*/
extern QueueHandle_t xUartRxQueue;

/*============================================================================
 *                          API实现
 *===========================================================================*/

/**
 * @brief  初始化UART4 MQTT接收（DMA模式）
 * @note   使用DMA1_Stream2接收数据，IDLE中断检测数据包结束
 */
void UART4_MQTT_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  DMA_InitTypeDef DMA_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  /*===== 1. 使能时钟 =====*/
  RCC_AHB1PeriphClockCmd(UART4_GPIO_CLK, ENABLE);      // GPIO时钟
  RCC_APB1PeriphClockCmd(UART4_CLK, ENABLE);           // UART4时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // DMA1时钟

  /*===== 2. GPIO配置 =====*/
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;      // 复用功能
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;    // 推挽输出
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;      // 上拉
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz

  /* TX引脚配置 */
  GPIO_InitStruct.GPIO_Pin = UART4_TX_PIN;
  GPIO_Init(UART4_TX_PORT, &GPIO_InitStruct);
  GPIO_PinAFConfig(UART4_TX_PORT, UART4_TX_SOURCE, UART4_AF);

  /* RX引脚配置 */
  GPIO_InitStruct.GPIO_Pin = UART4_RX_PIN;
  GPIO_Init(UART4_RX_PORT, &GPIO_InitStruct);
  GPIO_PinAFConfig(UART4_RX_PORT, UART4_RX_SOURCE, UART4_AF);

  /*===== 3. USART配置 =====*/
  USART_InitStruct.USART_BaudRate = UART4_BAUDRATE;        // 波特率
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // 8位数据
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // 1位停止位
  USART_InitStruct.USART_Parity = USART_Parity_No;         // 无校验
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
  USART_Init(UART4, &USART_InitStruct);

  /*===== 4. DMA配置 (DMA1_Stream2, Channel4 for UART4_RX) =====*/
  DMA_DeInit(DMA1_Stream2);

  DMA_InitStruct.DMA_Channel = DMA_Channel_4; // UART4_RX对应Channel4
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR); // 外设地址
  DMA_InitStruct.DMA_Memory0BaseAddr =
      (uint32_t)mqtt_rx_buffer[0]; // 初始指向 Buffer 0
  //  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)mqtt_rx_buffer;  //
  //  内存地址
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;          // 外设到内存
  DMA_InitStruct.DMA_BufferSize = MQTT_RX_BUF_SIZE;             // 缓冲区大小
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不增
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;          // 内存地址递增
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;          // Normal模式（非循环）
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;    // 高优先级
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable; // 禁用FIFO
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStruct);

  /*===== 5. 启用UART4的DMA接收请求 =====*/
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

  /*===== 6. NVIC中断配置 =====*/
  NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =
      6; // 抢占优先级6(FreeRTOS要求>=5)
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /*===== 7. 使能IDLE和ORE中断 =====*/
  USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); // IDLE中断（数据包结束检测）
  USART_ITConfig(UART4, USART_IT_ORE, ENABLE);  // ORE中断（溢出错误）

  /*===== 8. 启动DMA传输 =====*/
  DMA_Cmd(DMA1_Stream2, ENABLE);

  /*===== 9. 使能UART =====*/
  USART_Cmd(UART4, ENABLE);

  /* 首次清除IDLE标志位 (防止刚启动就进中断) */
  volatile uint32_t temp;
  temp = UART4->SR;
  temp = UART4->DR;
  (void)temp; // 避免编译器警告

  /* 调试：打印初始化后的DMA状态 */
  printf("[UART4] Init complete, checking DMA status...\r\n");
  // vTaskDelay(pdMS_TO_TICKS(100)); // ❌
  // 错误！调度器未启动前不能使用FreeRTOS延时！

  // 使用简单的忙等待代替 (大约100ms)
  for (volatile int i = 0; i < 1680000; i++) {
    __NOP();
  }

  UART4_PrintDMAStatus();
}

/**
 * @brief  获取UART4 DMA状态（调试用）
 * @note   可在任务中调用查看DMA是否正常工作
 */
void UART4_PrintDMAStatus(void) {
  uint16_t remaining = DMA_GetCurrDataCounter(DMA1_Stream2);
  uint32_t dma_enabled = (DMA1_Stream2->CR & DMA_SxCR_EN) ? 1 : 0;

  printf("[UART4-DMA] Enabled=%lu, Remaining=%u/%u, BufIdx=%u\r\n", dma_enabled,
         remaining, MQTT_RX_BUF_SIZE, g_rx_buf_index);
}

/*============================================================================
 *                          中断处理函数
 *===========================================================================*/

/**
 * @brief  UART4中断处理函数（DMA模式）
 * @note   处理IDLE和ORE中断，ORE必须处理否则USART会锁死
 *         关键：中断中不使用printf，避免阻塞导致UART溢出
 */
void UART4_IRQHandler(void) {
  uint32_t temp;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint32_t isr_count = 0; // 调试计数器

  isr_count++; // 调试：每次进中断计数

  // 检测 IDLE 中断
  if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET) {
    // 1. 清除 IDLE 标志（先清标志，后操作DMA）
    temp = UART4->SR;
    temp = UART4->DR;
    (void)temp;
    // 2. 计算本帧数据长度
    uint16_t len = MQTT_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2);

    // 3. 关闭DMA
    DMA_Cmd(DMA1_Stream2, DISABLE);

    // ⚠️ 强制复位DMA流（确保能成功重启）
    DMA_DeInit(DMA1_Stream2);

    // 4. 如果收到有效数据
    if (len > 0) {
      UartRxMsg_t msg;
      // 记录当前接收好的缓冲区地址和长度
      msg.pBuffer = mqtt_rx_buffer[g_rx_buf_index];
      msg.length = len;
      // 5. 切换到另一个缓冲区 (Ping-Pong)，来回切换
      g_rx_buf_index = !g_rx_buf_index;
      // 6. 重新配置DMA地址和计数器
      DMA1_Stream2->M0AR = (uint32_t)mqtt_rx_buffer[g_rx_buf_index];
      DMA_SetCurrDataCounter(DMA1_Stream2, MQTT_RX_BUF_SIZE);
      // 7. 发送消息到队列
      if (xUartRxQueue != NULL) {
        BaseType_t queueResult =
            xQueueSendFromISR(xUartRxQueue, &msg, &xHigherPriorityTaskWoken);
        (void)queueResult;
      }
    } else {
      // 0字节，只重置计数器
      DMA_SetCurrDataCounter(DMA1_Stream2, MQTT_RX_BUF_SIZE);
    }

    // 8. 重新初始化DMA并启动
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
    DMA_InitStruct.DMA_Memory0BaseAddr =
        (uint32_t)mqtt_rx_buffer[g_rx_buf_index];
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = MQTT_RX_BUF_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &DMA_InitStruct);

    // 9. 启动DMA
    DMA_Cmd(DMA1_Stream2, ENABLE);
  }

  // 处理 ORE
  if (USART_GetITStatus(UART4, USART_IT_ORE) != RESET) {
    temp = UART4->SR;
    temp = UART4->DR;
    (void)temp;
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
