#include "sal.h"
#include "modbus.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// SAL（盐度）传感器驱动实现
//
// 功能说明：
//   SAL传感器用于测量水体的盐度
//   盐度反映水中溶解盐类的含量，单位为ppt（千分之）
//
// 数据解析位置：
//   SAL数据（0x2602）：
//   - SAL值： byte[0-3]  (第1-4字节，IEEE754浮点数)
//
// Modbus寄存器：
//   - 0x2602: SAL数据地址（2个寄存器=4字节）
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define SAL_ADDR 0x06            // Modbus从机地址
#define SAL_REG_DATA_ADDR 0x2602 // SAL数据寄存器地址
#define SAL_REG_DATA_NUM 0x02    // 寄存器数量（2个=4字节）

/**
 * @brief  读取盐度值
 * @param  sal_value: 输出参数，盐度值（ppt）
 * @return true:读取成功, false:读取失败
 */
bool sal_read(float *sal_value) {
  uint8_t raw_data[10];
  uint16_t data_len;

  /* 发送Modbus读取命令：读取0x2602寄存器，2个寄存器 */
  if (!modbus_read_holding(SAL_ADDR, SAL_REG_DATA_ADDR, SAL_REG_DATA_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  /* 检查数据长度，应为4字节 */
  if (data_len >= 4) {
    /* 将4字节转换为IEEE754浮点数 */
    *sal_value = modbus_bytes_to_float(&raw_data[0]);
    return true;
  }

  return false;
}
