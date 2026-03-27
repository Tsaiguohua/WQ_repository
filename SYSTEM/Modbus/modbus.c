#include "modbus.h"
#include "../RS485/rs485.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// Modbus RTU主站协议层实现
//
// 功能说明：
//   实现Modbus RTU主站（Master）功能，用于与水质传感器通信
//   支持功能码03（读保持寄存器）和10（写多个寄存器）
//
// Modbus RTU帧格式：
//   请求帧: [从机地址1B][功能码1B][数据NB][CRC_L][CRC_H]
//   响应帧: [从机地址1B][功能码1B][数据NB][CRC_L][CRC_H]
//
// CRC校验：
//   Modbus RTU标准：低字节在前，高字节在后（Little Endian）
//   使用查表法计算CRC16-Modbus，比逐位计算快约10倍
//
// 数据转换：
//   传感器返回的float数据是小端IEEE754格式（LSB在前，MSB在后）
//   STM32F407也是小端架构，因此不需要字节序转换
//   直接将接收到的4个字节按顺序赋值给float即可
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// Modbus RTU主站协议层 - FreeRTOS版本
//
// 功能说明：
//   本模块实现Modbus RTU主站（Master）功能，用于STM32与水质传感器通信。
//   基于RS485物理层，提供标准Modbus RTU协议支持：
//   - 功能码03：读保持寄存器（Read Holding Registers）
//   - 功能码10（0x10）：写多个寄存器（Write Multiple Registers）
//   - CRC16校验确保数据完整性
//   - 支持IEEE754浮点数字节序转换
//
// Modbus RTU协议详解：
//   【帧格式】
//   请求帧: [从机地址1B][功能码1B][数据NB][CRC_L][CRC_H]
//   响应帧: [从机地址1B][功能码1B][数据NB][CRC_L][CRC_H]
//
//   【功能码03 - 读保持寄存器】
//   请求：[地址][03][寄存器地址H][地址L][数量H][数量L][CRC_L][CRC_H]
//   响应：[地址][03][字节数][数据...][CRC_L][CRC_H]
//
//   【功能码10 - 写多个寄存器】
//   请求：[地址][10][寄存器地址H][地址L][数量H][数量L][字节数][数据...][CRC_L][CRC_H]
//   响应：[地址][10][寄存器地址H][地址L][数量H][数量L][CRC_L][CRC_H]
//
//   【CRC16校验】
//   - 算法：CRC16-Modbus（多项式0xA001）
//   - 初始值：0xFFFF
//   - ⚠️ 字节序：低字节在前，高字节在后（Little Endian）
//   - 示例：CRC=0x9474 → 传输顺序 `74 94`
//   - 使用查表法计算，比逐位计算快约10倍
//
//   【数据字段】
//   - 寄存器地址（2字节）：高字节在前（Big Endian）
//   - 寄存器数量（2字节）：高字节在前（Big Endian）
//   - ⚠️ 浮点数（4字节）：传感器返回小端序（LSB在前），STM32也是小端序，无需转换
//
// 数据转换：
//   【IEEE754浮点数转换】
//   - 传感器返回：小端序（LSB在前）例如：`B2 C2 C0 42`
//   - STM32内存：小端序（LSB在前）例如：`B2 C2 C0 42`
//   - 结论：✅ 字节序相同，直接赋值，不需要反转！
//   - 函数 modbus_bytes_to_float() 直接拷贝字节，无需反转
//
//   ⚠️ 修正说明（2026/01/15）：
//   传感器返回的就是小端序，与STM32架构一致，直接赋值即可。
//
// API接口：
//   - modbus_init()：初始化Modbus（内部调用rs485_init）
//   - modbus_read_holding()：读保持寄存器，返回数据
//   - modbus_write_multiple()：写多个寄存器
//   - modbus_bytes_to_float()：4字节转IEEE754浮点数（小端序，无需反转）
//   - modbus_crc16()：计算CRC16校验码
//
// 硬件依赖：
//   - 底层：RS485通信层（rs485.c）
//   - 串口：USART2（PA2/PA3，9600波特率）
//   - 物理：RS485转TTL模块（自动收发切换，无需DE/RE控制）
//
// FreeRTOS安全：
//   - 内部使用RS485互斥锁（rs485_mutex）保护总线访问
//   - 支持多任务并发调用，同一时刻只有一个任务访问总线
//   - 超时机制防止死锁（默认1000ms）
//
// 注意事项：
//   ⚠️ CRC字节序：Modbus RTU标准是**低字节在前**
//   ⚠️ 数据字段：寄存器地址和数量是**高字节在前**（大端序）
//   ⚠️ 浮点数字节序：传感器返回**小端序**，STM32是**小端序**，直接赋值，不反转！
//   ⚠️ 必须在FreeRTOS启动后调用（内部使用信号量）
//
// 作者：蔡国华
// 创建日期：2025/12/31
// 最后更新：2026/01/15
// 版本：v1.2（修正浮点数字节序说明）
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          模块私有变量
 *===========================================================================*/

/* 发送缓冲区 */
static uint8_t modbus_tx_buf[64];

/* 接收缓冲区 */
static uint8_t modbus_rx_buf[128];

/*============================================================================
 *                          CRC16查表（私有）
 *
 * CRC16-Modbus算法：
 *   - 初始值: 0xFFFF
 *   - 多项式: 0x8005（反转后0xA001）
 *   - 使用查表法加速计算
 *===========================================================================*/

/* CRC16查找表：预计算的256个CRC值 */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601,
    0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0,
    0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81,
    0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0,
    0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081,
    0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00,
    0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700,
    0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
    0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01,
    0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1,
    0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101,
    0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481,
    0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
    0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581,
    0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040};

/**
 * @brief  计算CRC16校验值
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @return CRC16值
 * @note   使用查表法，比逐位计算快约10倍
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF; // 初始值

  while (len--) {
    /* 查表计算：crc = (crc >> 8) ^ table[(crc ^ data) & 0xFF] */
    crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF];
  }

  return crc;
}

/*============================================================================
 *                          数据转换（私有）
 *===========================================================================*/

/**
 * @brief  4字节转换为float(IEEE754格式)
 * @param  bytes: 4字节数组指针
 * @return 浮点数值
 * @note   参考裸机版本的 floatTOdecimal 函数
 *         裸机调用: floatTOdecimal(RS485_RX_BUFF[7], RS485_RX_BUFF[8],
 * RS485_RX_BUFF[9], RS485_RX_BUFF[10]) 参数顺序: byte0(最低字节), byte1, byte2,
 * byte3(最高字节) Modbus响应格式:
 * [地址][功能码][字节数][数据0][数据1][数据2][数据3]... 即: bytes[0]=最低字节,
 * bytes[3]=最高字节(小端序)
 */
float modbus_bytes_to_float(uint8_t *bytes) {
  /* 输入验证 */
  if (bytes == NULL) {
    return 0.0f;
  }

  /* 使用union进行类型转换 */
  union {
    uint8_t b[4];
    float f;
  } data;

  /* 初始化为0,防止垃圾数据 */
  data.f = 0.0f;

  /* STM32是小端序,传感器返回也是小端序,直接赋值不反转
   * bytes[0] = 最低字节(LSB)
   * bytes[3] = 最高字节(MSB)
   */
  data.b[0] = bytes[0]; // 最低字节
  data.b[1] = bytes[1];
  data.b[2] = bytes[2];
  data.b[3] = bytes[3]; // 最高字节

  /* 验证浮点数的合理性,防止NaN或无穷大 */
  if (data.f != data.f) { // NaN检测: NaN != NaN
    return 0.0f;
  }

  /* 检查是否为无穷大或超出合理范围 */
  if (data.f > 1.0e10f || data.f < -1.0e10f) {
    return 0.0f;
  }

  return data.f;
}

/*============================================================================
 *                          公开API实现
 *===========================================================================*/

/**
 * @brief  初始化Modbus
 * @return true:成功, false:失败
 * @note   内部会初始化RS485通信层
 */
bool modbus_init(void) { return rs485_init(); }

/**
 * @brief  读取保持寄存器（功能码03）
 * @param  slave_addr: 从机地址（如0x07表示Y4000）
 * @param  reg_addr: 寄存器起始地址（如0x2601）
 * @param  reg_num: 要读取的寄存器数量
 * @param  data: 输出缓冲区
 * @param  data_len: 输出数据长度
 * @return true:成功（CRC校验通过）, false:失败
 *
 * 请求帧格式（8字节）：
 *   [从机地址][03][寄存器地址H][地址L][数量H][数量L][CRC_H][CRC_L]
 *
 * 响应帧格式：
 *   [从机地址][03][字节数][数据...][CRC_H][CRC_L]
 */
bool modbus_read_holding(uint8_t slave_addr, uint16_t reg_addr,
                         uint16_t reg_num, uint8_t *data, uint16_t *data_len) {
  uint16_t crc;
  uint16_t rx_len;
  uint16_t rx_crc, calc_crc;

  /*===== 1. 构建请求帧 =====*/
  modbus_tx_buf[0] = slave_addr;               // 从机地址
  modbus_tx_buf[1] = MODBUS_FUNC_READ_HOLDING; // 功能码03
  modbus_tx_buf[2] = (reg_addr >> 8) & 0xFF;   // 寄存器地址高字节
  modbus_tx_buf[3] = reg_addr & 0xFF;          // 寄存器地址低字节
  modbus_tx_buf[4] = (reg_num >> 8) & 0xFF;    // 寄存器数量高字节
  modbus_tx_buf[5] = reg_num & 0xFF;           // 寄存器数量低字节

  /* 计算并添加CRC（Modbus RTU: 低字节在前） */
  crc = modbus_crc16(modbus_tx_buf, 6);
  modbus_tx_buf[6] = crc & 0xFF;        // CRC低字节在前
  modbus_tx_buf[7] = (crc >> 8) & 0xFF; // CRC高字节在后

  /*===== 2. 发送请求并等待响应 =====*/
  if (!rs485_transfer(modbus_tx_buf, 8, modbus_rx_buf, &rx_len,
                      MODBUS_DEFAULT_TIMEOUT_MS)) {
    return false; // 通信失败
  }

  /*===== 3. 检查响应 =====*/
  /* 最小长度检查：地址(1) + 功能码(1) + 字节数(1) + CRC(2) = 5 */
  if (rx_len < 5) {
    return false;
  }

  /* 检查从机地址 */
  if (modbus_rx_buf[0] != slave_addr) {
    return false;
  }

  /* 检查功能码（异常响应时功能码最高位为1） */
  if (modbus_rx_buf[1] != MODBUS_FUNC_READ_HOLDING) {
    return false;
  }

  /*===== 4. CRC校验（Modbus RTU: 低字节在前） =====*/
  calc_crc = modbus_crc16(modbus_rx_buf, rx_len - 2);
  rx_crc = modbus_rx_buf[rx_len - 2] |
           (modbus_rx_buf[rx_len - 1] << 8); // 低字节在前
  if (calc_crc != rx_crc) {
    return false; // CRC错误
  }

  /*===== 5. 提取数据 =====*/
  /* 响应格式: [地址][功能码][字节数][数据...][CRC] */
  uint8_t byte_count = modbus_rx_buf[2];
  if (byte_count > 0 && data != NULL) {
    memcpy(data, &modbus_rx_buf[3], byte_count);
    *data_len = byte_count;
  }

  return true;
}

/**
 * @brief  写多个寄存器（功能码10）
 * @param  slave_addr: 从机地址
 * @param  reg_addr: 寄存器起始地址
 * @param  reg_num: 寄存器数量
 * @param  data: 要写入的数据
 * @param  data_len: 数据长度（字节）
 * @return true:成功, false:失败
 *
 * 请求帧格式：
 *   [从机地址][10][寄存器地址H][地址L][数量H][数量L][字节数][数据...][CRC]
 *
 * 响应帧格式（8字节）：
 *   [从机地址][10][寄存器地址H][地址L][数量H][数量L][CRC_H][CRC_L]
 */
bool modbus_write_multiple(uint8_t slave_addr, uint16_t reg_addr,
                           uint16_t reg_num, uint8_t *data, uint16_t data_len) {
  uint16_t crc;
  uint16_t rx_len;
  uint16_t frame_len;

  /*===== 1. 构建请求帧 =====*/
  modbus_tx_buf[0] = slave_addr;                 // 从机地址
  modbus_tx_buf[1] = MODBUS_FUNC_WRITE_MULTIPLE; // 功能码10
  modbus_tx_buf[2] = (reg_addr >> 8) & 0xFF;     // 寄存器地址高字节
  modbus_tx_buf[3] = reg_addr & 0xFF;            // 寄存器地址低字节
  modbus_tx_buf[4] = (reg_num >> 8) & 0xFF;      // 寄存器数量高字节
  modbus_tx_buf[5] = reg_num & 0xFF;             // 寄存器数量低字节
  modbus_tx_buf[6] = data_len;                   // 字节数

  /* 复制数据 */
  if (data != NULL && data_len > 0) {
    memcpy(&modbus_tx_buf[7], data, data_len);
  }

  /* 计算帧长度并添加CRC（Modbus RTU: 低字节在前） */
  frame_len = 7 + data_len;
  crc = modbus_crc16(modbus_tx_buf, frame_len);
  modbus_tx_buf[frame_len] = crc & 0xFF;            // CRC低字节在前
  modbus_tx_buf[frame_len + 1] = (crc >> 8) & 0xFF; // CRC高字节在后

  /*===== 2. 发送请求并等待响应 =====*/
  if (!rs485_transfer(modbus_tx_buf, frame_len + 2, modbus_rx_buf, &rx_len,
                      MODBUS_DEFAULT_TIMEOUT_MS)) {
    return false;
  }

  /*===== 3. 检查响应 =====*/
  if (rx_len < 8) {
    return false;
  }

  if (modbus_rx_buf[0] != slave_addr ||
      modbus_rx_buf[1] != MODBUS_FUNC_WRITE_MULTIPLE) {
    return false;
  }

  return true;
}
