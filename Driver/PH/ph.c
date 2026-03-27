#include "ph.h"
#include "modbus.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// PH传感器驱动实现
//
// 功能说明：
//   PH传感器用于测量水体的酸碱度
//   PH值范围：0-14（7为中性，<7为酸性，>7为碱性）
//
// 数据解析位置：
//   PH数据（0x2800）：
//   - PH值： byte[0-3]  (第1-4字节，IEEE754浮点数)
//
// Modbus寄存器：
//   - 0x2800: PH数据地址（2个寄存器=4字节）
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define PH_ADDR 0x04            // Modbus从机地址
#define PH_REG_DATA_ADDR 0x2800 // PH数据寄存器地址
#define PH_REG_DATA_NUM 0x02    // 寄存器数量（2个=4字节）

/**
 * @brief  读取PH值
 * @param  ph_value: 输出参数，PH值（0-14）
 * @return true:读取成功, false:读取失败
 * @note   PH=7为中性，<7为酸性，>7为碱性
 */
bool ph_read(float *ph_value) {
  uint8_t raw_data[10];
  uint16_t data_len;

  /* 发送Modbus读取命令：读取0x2800寄存器，2个寄存器 */
  if (!modbus_read_holding(PH_ADDR, PH_REG_DATA_ADDR, PH_REG_DATA_NUM, raw_data,
                           &data_len)) {
    return false;
  }

  /* 检查数据长度，应为4字节 */
  if (data_len >= 4) {
    /* 将4字节转换为IEEE754浮点数 */
    *ph_value = modbus_bytes_to_float(&raw_data[0]);
    return true;
  }

  return false;
}
