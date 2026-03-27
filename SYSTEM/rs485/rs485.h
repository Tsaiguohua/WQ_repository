#ifndef RS485_H
#define RS485_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// RS485通信层 - 面向对象风格
//
// 功能说明：
//   1. 基于USART2实现RS485半双工通信
//   2. 使用FreeRTOS信号量实现多任务互斥访问
//   3. 支持发送、接收、收发一体三种操作模式
//
// 硬件连接：
//   - TX: PA2
//   - RX: PA3
//   - 波特率: 9600
//
// 使用方法：
//   1. 调用 rs485_init() 初始化
//   2. 使用 rs485_transfer() 进行Modbus通信
//
// 目标平台: STM32F407ZGT6
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/* RS485缓冲区大小定义 */
#define RS485_TX_BUF_SIZE 64  // 发送缓冲区大小（字节）
#define RS485_RX_BUF_SIZE 128 // 接收缓冲区大小（字节）

/**
 * @brief  初始化RS485通信
 * @note   包括GPIO、USART、NVIC初始化，以及创建FreeRTOS信号量
 * @return true:初始化成功, false:初始化失败（信号量创建失败）
 */
bool rs485_init(void);

/**
 * @brief  RS485发送数据
 * @param  data: 待发送数据的指针
 * @param  len: 数据长度（字节）
 * @note   阻塞式发送，等待发送完成后返回
 * @return true:发送成功, false:参数错误
 */
bool rs485_send(uint8_t *data, uint16_t len);

/**
 * @brief  RS485接收数据（带超时）
 * @param  buf: 接收缓冲区指针
 * @param  len: 输出参数，返回实际接收到的字节数
 * @param  timeout_ms: 超时时间（毫秒），超时后返回已接收的数据
 * @note   使用FreeRTOS信号量等待，超时期间会让出CPU
 * @return true:接收到数据, false:超时且无数据
 */
bool rs485_receive(uint8_t *buf, uint16_t *len, uint32_t timeout_ms);

/**
 * @brief  RS485收发一体操作（最常用）
 * @param  tx_data: 发送数据指针
 * @param  tx_len: 发送数据长度
 * @param  rx_buf: 接收缓冲区指针
 * @param  rx_len: 输出参数，返回接收到的字节数
 * @param  timeout_ms: 接收超时时间（毫秒）
 * @note   内部会获取互斥锁，保证同一时刻只有一个任务访问RS485总线
 *         典型应用：Modbus请求-响应通信
 * @return true:收发成功, false:失败（获取锁超时或通信超时）
 */
bool rs485_transfer(uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_buf,
                    uint16_t *rx_len, uint32_t timeout_ms);

#endif
