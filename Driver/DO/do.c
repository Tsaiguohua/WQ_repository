#include "do.h"
#include "modbus.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// DO（溶解氧）传感器驱动实现
//
// 功能说明：
//   DO传感器用于测量水体中的溶解氧含量
//   溶解氧是评估水质的重要指标，影响水生生物生存
//
// 数据解析位置：
//   DO数据（0x2600）：
//   - DO值： byte[8-11]  (第9-12字节，IEEE754浮点数)
//
// Modbus寄存器：
//   - 0x2600: DO数据地址（6个寄存器=12字节）
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define DO_ADDR 0x05            // Modbus从机地址
#define DO_REG_DATA_ADDR 0x2600 // DO数据寄存器地址
#define DO_REG_DATA_NUM 0x06    // 寄存器数量（6个=12字节）

/**
 * @brief  读取溶解氧值
 * @param  do_value: 输出参数，DO值（mg/L）
 * @return true:读取成功, false:读取失败
 * @note   DO值位于返回数据的byte[8-11]位置
 */
bool do_read(float *do_value) {
  uint8_t raw_data[20];
  uint16_t data_len;

  /* 发送Modbus读取命令：读取0x2600寄存器，6个寄存器 */
  if (!modbus_read_holding(DO_ADDR, DO_REG_DATA_ADDR, DO_REG_DATA_NUM, raw_data,
                           &data_len)) {
    return false;
  }

  /* 检查数据长度，应至少为12字节 */
  if (data_len >= 12) {
    /* DO值在byte[8-11]位置（和裸机一致） */
    *do_value = modbus_bytes_to_float(&raw_data[8]);
    return true;
  }

  return false;
}
