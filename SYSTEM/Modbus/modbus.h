#ifndef MODBUS_H
#define MODBUS_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// Modbus RTU主站协议层 - 面向对象风格
//
// 功能说明：
//   1. 实现Modbus RTU主站功能
//   2. 支持功能码03（读保持寄存器）和10（写多个寄存器）
//   3. 内置CRC16校验（查表法，高效）
//   4. 提供IEEE754浮点数转换
//
// 协议格式（Modbus RTU）：
//   请求帧: [从机地址][功能码][寄存器地址H][地址L][数量H][数量L][CRC_H][CRC_L]
//   响应帧: [从机地址][功能码][字节数][数据...][CRC_H][CRC_L]
//
// 使用方法：
//   1. 调用 modbus_init() 初始化（会初始化底层RS485）
//   2. 使用 modbus_read_holding() 读取传感器数据
//   3. 使用 modbus_bytes_to_float() 将4字节转换为浮点数
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/* Modbus功能码定义 */
#define MODBUS_FUNC_READ_HOLDING 0x03   // 读保持寄存器（最常用）
#define MODBUS_FUNC_WRITE_MULTIPLE 0x10 // 写多个寄存器

/* 超时配置 */
#define MODBUS_DEFAULT_TIMEOUT_MS 300 // 默认响应超时（毫秒）

/**
 * @brief  Modbus初始化
 * @note   内部会调用rs485_init()初始化底层通信
 * @return true:成功, false:失败
 */
bool modbus_init(void);

/**
 * @brief  读取保持寄存器（功能码03）
 * @param  slave_addr: 从机地址（如：0x01=COD, 0x07=Y4000）
 * @param  reg_addr: 寄存器起始地址（如：0x2600）
 * @param  reg_num: 要读取的寄存器数量
 * @param  data: 输出缓冲区，存放接收到的寄存器数据
 * @param  data_len: 输出参数，返回实际接收的字节数
 * @note   这是读取传感器数据的核心函数
 *         内部完成：构建请求帧 -> 发送 -> 等待响应 -> CRC校验 -> 提取数据
 * @return true:读取成功（CRC校验通过）, false:失败
 *
 * @example
 *   uint8_t data[20];
 *   uint16_t len;
 *   if (modbus_read_holding(0x07, 0x2601, 0x12, data, &len)) {
 *       // 解析data中的传感器数据
 *   }
 */
bool modbus_read_holding(uint8_t slave_addr, uint16_t reg_addr,
                         uint16_t reg_num, uint8_t *data, uint16_t *data_len);

/**
 * @brief  写多个寄存器（功能码10）
 * @param  slave_addr: 从机地址
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_num: 寄存器数量
 * @param  data: 要写入的数据
 * @param  data_len: 数据长度（字节）
 * @note   用于设置传感器参数、开启刷子等操作
 * @return true:写入成功, false:失败
 */
bool modbus_write_multiple(uint8_t slave_addr, uint16_t reg_addr,
                           uint16_t reg_num, uint8_t *data, uint16_t data_len);

/**
 * @brief  4字节数据转换为float（IEEE754格式）
 * @param  bytes: 4字节数组指针
 * @note   传感器返回的浮点数是小端格式，此函数会自动处理字节序
 *         水质传感器的温度、PH、DO、COD等数值都是float类型
 * @return 转换后的浮点数值
 *
 * @example
 *   float ph_value = modbus_bytes_to_float(&raw_data[12]);
 */
float modbus_bytes_to_float(uint8_t *bytes);

/**
 * @brief  计算Modbus CRC16校验值
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @note   使用查表法实现，比计算法快很多
 * @return CRC16值（高字节在前）
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t len);

#endif
