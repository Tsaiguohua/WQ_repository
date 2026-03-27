#include "rs485.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// RS485通信底层驱动 - FreeRTOS版本
//
// 功能说明：
//   本模块实现RS485半双工通信的底层驱动，为Modbus协议层提供可靠的数据收发功能。
//   基于STM32F407的USART2，使用FreeRTOS信号量实现多任务安全通信。
//
// 硬件配置：
//   【串口配置】
//   - 串口：USART2（APB1总线）
//   - TX引脚：PA2（复用推挽输出）
//   - RX引脚：PA3（复用浮空输入）
//   - 波特率：9600（与传感器波特率一致）
//   - 数据位：8位
//   - 停止位：1位
//   - 校验位：无
//   - 流控：无
//
//   【RS485模块】
//   - 类型：自动收发切换RS485转TTL模块
//   - 无需DE/RE控制引脚（模块内部自动切换）
//   - 总线：半双工，同一时刻只能发送或接收
//
// 通信机制：
//   【发送流程】
//   1. 获取总线互斥锁（rs485_mutex）
//   2. 逐字节发送数据到USART2
//   3. 等待发送完成（TC标志）
//   4. 返回（互斥锁在rs485_transfer中释放）
//
//   【接收流程】
//   1. USART2 RXNE中断：逐字节接收数据
//   2. USART2 IDLE中断：检测帧结束（总线空闲）
//   3. IDLE触发后，释放接收信号量（rs485_rx_sem）
//   4. 上层任务被唤醒，读取接收数据
//
//   【帧结束检测】
//   - 使用USART IDLE中断检测总线空闲（一个字节时间无数据）
//   - IDLE中断比定时器轮询更准确、更高效
//   - 接收完成后通过信号量通知任务
//
// FreeRTOS同步：
//   【互斥锁（rs485_mutex）】
//   - 用途：保护RS485总线，防止多任务同时访问
//   - 超时：1秒（避免死锁）
//   - 类型：Mutex（支持优先级继承）
//
//   【信号量（rs485_rx_sem）】
//   - 用途：接收完成通知
//   - 类型：Binary Semaphore
//   - 触发：IDLE中断中释放
//
// 中断处理：
//   【USART2_IRQHandler】
//   - ⚠️ 函数名必须与startup文件中的向量表一致！
//   - 错误的函数名会导致中断无法进入
//   - 优先级：6（≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5）
//   - 处理：RXNE（逐字节接收）+ IDLE（帧结束检测）
//
// API接口：
//   - rs485_init()：初始化RS485（GPIO、USART、中断、信号量）
//   - rs485_send()：发送数据（阻塞式，等待发送完成）
//   - rs485_receive()：接收数据（带超时，使用信号量等待）
//   - rs485_transfer()：收发一体（最常用，自动加锁）
//
// 注意事项：
//   ⚠️ 中断函数名必须是 USART2_IRQHandler，不能自定义
//   ⚠️ 中断优先级必须 ≥ 5，否则FreeRTOS API会失败
//   ⚠️ 必须在FreeRTOS启动后调用rs485_init（创建信号量需要堆内存）
//   ⚠️ 不要在中断中直接调用printf（堆栈消耗大）
//   ⚠️ RS485是半双工，发送和接收不能同时进行
//
// 调试技巧：
//   - 如果收不到数据：检查中断函数名是否正确（USART2_IRQHandler）
//   - 如果CRC错误：检查字节序（Modbus RTU是低字节在前）
//   - 如果超时：增加接收超时时间，或检查硬件连接
//
// 作者：蔡国华
// 创建日期：2025/12/31
// 最后更新：2026/01/14
// 版本：v1.1（修复中断函数名错误）
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          硬件配置宏定义
 * 根据你的实际硬件连接修改这些宏
 *===========================================================================*/

#define RS485_USART USART2                       // 使用的USART
#define RS485_USART_CLK RCC_APB1Periph_USART2    // USART时钟
#define RS485_USART_IRQn USART2_IRQn             // 中断号
#define RS485_USART_IRQHandler USART2_IRQHandler // 中断处理函数名

#define RS485_GPIO_CLK RCC_AHB1Periph_GPIOA // GPIO时钟
#define RS485_TX_PORT GPIOA                 // TX端口
#define RS485_TX_PIN GPIO_Pin_2             // TX引脚 (PA2)
#define RS485_TX_SOURCE GPIO_PinSource2     // TX引脚源
#define RS485_RX_PORT GPIOA                 // RX端口
#define RS485_RX_PIN GPIO_Pin_3             // RX引脚 (PA3)
#define RS485_RX_SOURCE GPIO_PinSource3     // RX引脚源
#define RS485_AF GPIO_AF_USART2             // 复用功能

#define RS485_BAUDRATE 9600 // 波特率

/*============================================================================
 *                          模块私有变量
 *===========================================================================*/

/* 接收缓冲区 */
static uint8_t rs485_rx_buf[RS485_RX_BUF_SIZE]; // 接收数据缓冲区
static volatile uint16_t rs485_rx_cnt = 0;      // 接收计数器
static volatile bool rs485_rx_complete = false; // 接收完成标志

/* FreeRTOS同步对象 */
static SemaphoreHandle_t rs485_mutex = NULL; // 总线互斥锁（防止多任务同时访问）
static SemaphoreHandle_t rs485_rx_sem = NULL; // 接收完成信号量

/* 帧间隔超时检测 */
static volatile uint32_t rs485_last_rx_tick = 0; // 最后接收时间
#define RS485_FRAME_TIMEOUT_MS 10                // 10ms无新数据认为一帧结束

/*============================================================================
 *                          硬件初始化（私有函数）
 *===========================================================================*/

/**
 * @brief  GPIO和USART硬件初始化
 * @note   配置TX/RX引脚、USART参数、中断
 */
static void rs485_hw_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  /*===== 1. 使能时钟 =====*/
  RCC_AHB1PeriphClockCmd(RS485_GPIO_CLK, ENABLE);  // GPIO时钟
  RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE); // USART时钟

  /*===== 2. GPIO配置 =====*/
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;      // 复用功能
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;    // 推挽输出
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;      // 上拉
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz

  /* TX引脚配置 */
  GPIO_InitStruct.GPIO_Pin = RS485_TX_PIN;
  GPIO_Init(RS485_TX_PORT, &GPIO_InitStruct);
  GPIO_PinAFConfig(RS485_TX_PORT, RS485_TX_SOURCE, RS485_AF);

  /* RX引脚配置 */
  GPIO_InitStruct.GPIO_Pin = RS485_RX_PIN;
  GPIO_Init(RS485_RX_PORT, &GPIO_InitStruct);
  GPIO_PinAFConfig(RS485_RX_PORT, RS485_RX_SOURCE, RS485_AF);

  /*===== 3. USART配置 =====*/
  USART_InitStruct.USART_BaudRate = RS485_BAUDRATE;        // 波特率
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // 8位数据
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // 1位停止位
  USART_InitStruct.USART_Parity = USART_Parity_No;         // 无校验
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
  USART_Init(RS485_USART, &USART_InitStruct);

  /*===== 4. NVIC中断配置 =====*/
  NVIC_InitStruct.NVIC_IRQChannel = RS485_USART_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =
      6; // 抢占优先级（FreeRTOS要求>=5）
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /*===== 5. 使能中断 =====*/
  USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE); // 接收中断
  USART_ITConfig(RS485_USART, USART_IT_IDLE, ENABLE); // 空闲中断（帧结束检测）

  /*===== 6. 使能USART =====*/
  USART_Cmd(RS485_USART, ENABLE);
}

/*============================================================================
 *                          公开API实现
 *===========================================================================*/

/**
 * @brief  初始化RS485
 * @return true:成功, false:失败（信号量创建失败）
 */
bool rs485_init(void) {
  /* 创建互斥锁：保护RS485总线，同一时刻只能一个任务访问 */
  rs485_mutex = xSemaphoreCreateMutex();

  /* 创建二值信号量：用于接收完成通知 */
  rs485_rx_sem = xSemaphoreCreateBinary();

  if (rs485_mutex == NULL || rs485_rx_sem == NULL) {
    return false; // 内存不足
  }

  /* 硬件初始化 */
  rs485_hw_init();

  return true;
}

/**
 * @brief  发送数据
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @return true:成功
 * @note   阻塞式发送，逐字节发送
 */
bool rs485_send(uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0)
    return false;

  /* 逐字节发送 */
  for (uint16_t i = 0; i < len; i++) {
    /* 等待发送缓冲区空 */
    while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET)
      ;
    USART_SendData(RS485_USART, data[i]);
  }

  /* 等待最后一个字节发送完成 */
  while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET)
    ;

  return true;
}

/**
 * @brief  接收数据（带超时）
 * @param  buf: 接收缓冲区
 * @param  len: 输出接收长度
 * @param  timeout_ms: 超时时间
 * @return true:接收到数据, false:超时
 */
bool rs485_receive(uint8_t *buf, uint16_t *len, uint32_t timeout_ms) {
  if (buf == NULL || len == NULL)
    return false;

  /* 清空接收状态 */
  rs485_rx_cnt = 0;
  rs485_rx_complete = false;
  memset(rs485_rx_buf, 0, RS485_RX_BUF_SIZE);

  /* 等待接收完成信号量（会让出CPU给其他任务） */
  if (xSemaphoreTake(rs485_rx_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
    /* 复制接收到的数据 */
    memcpy(buf, rs485_rx_buf, rs485_rx_cnt);
    *len = rs485_rx_cnt;
    return true;
  }

  /* 超时，但可能有部分数据 */
  if (rs485_rx_cnt > 0) {
    memcpy(buf, rs485_rx_buf, rs485_rx_cnt);
    *len = rs485_rx_cnt;
    return true;
  }

  *len = 0;
  return false;
}

/**
 * @brief  收发一体操作（最常用）
 * @param  tx_data: 发送数据
 * @param  tx_len: 发送长度
 * @param  rx_buf: 接收缓冲区
 * @param  rx_len: 输出接收长度
 * @param  timeout_ms: 超时时间
 * @return true:成功, false:失败
 * @note   内部获取互斥锁，保证总线访问安全
 */
bool rs485_transfer(uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_buf,
                    uint16_t *rx_len, uint32_t timeout_ms) {
  bool ret = false;

  /* 获取总线互斥锁（最多等待1秒） */
  if (xSemaphoreTake(rs485_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return false; // 获取锁失败
  }

  /* 清空接收状态 */
  rs485_rx_cnt = 0;
  rs485_rx_complete = false;

  /* 发送数据 */
  if (!rs485_send(tx_data, tx_len)) {
    xSemaphoreGive(rs485_mutex);
    return false;
  }

  /* 等待接收响应 */
  ret = rs485_receive(rx_buf, rx_len, timeout_ms);

  /* 释放互斥锁 */
  xSemaphoreGive(rs485_mutex);

  return ret;
}

/*============================================================================
 *                          中断处理函数
 *===========================================================================*/

/**
 * @brief  USART2中断处理函数
 * @note   处理RXNE（接收）和IDLE（空闲）中断
 *         ⚠️ 函数名必须与startup文件中的向量表一致！
 */
void USART2_IRQHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /*===== 接收中断：逐字节接收 =====*/
  if (USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET) {
    uint8_t data = USART_ReceiveData(RS485_USART);

    /* 存入缓冲区 */
    if (rs485_rx_cnt < RS485_RX_BUF_SIZE) {
      rs485_rx_buf[rs485_rx_cnt++] = data;
    }

    /* 记录最后接收时间 */
    rs485_last_rx_tick = xTaskGetTickCountFromISR();
  }

  /*===== 空闲中断：检测帧结束 =====*/
  if (USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET) {
    /* 清除IDLE标志：先读SR再读DR */
    (void)RS485_USART->SR;
    (void)RS485_USART->DR;

    /* 如果有数据，通知接收完成 */
    if (rs485_rx_cnt > 0) {
      rs485_rx_complete = true;
      xSemaphoreGiveFromISR(rs485_rx_sem, &xHigherPriorityTaskWoken);
    }
  }

  /* 如果需要，进行任务切换 */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
