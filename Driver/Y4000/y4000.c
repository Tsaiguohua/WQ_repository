#include "y4000.h"
#include "modbus.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// Y4000多参数传感器驱动实现
//
// 功能说明：
//   Y4000是多参数传感器母体，可一次性读取多个水质参数
//   包括PH、DO（溶解氧）、SAL（盐度）等
//
// 数据解析位置：
//   - DO值：  byte[0-3]   (第1-4字节)
//   - PH值：  byte[12-15] (第13-16字节)
//   - SAL值： byte[32-35] (第33-36字节)
//
// Modbus寄存器：
//   - 0x2601: DO/PH/SAL数据（读18个寄存器=36字节）
//   - 0x1600: 气压数据
//   - 0x2F00: 开启刷子命令
//   - 0x3000: 刷子间隔时间
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define Y4000_ADDR 0x07 // Modbus从机地址

#define Y4000_REG_DPS_ADDR 0x2601   // DO/PH/SAL数据起始地址
#define Y4000_REG_DPS_NUM 0x12      // 寄存器数量（18个）
#define Y4000_REG_ATM_ADDR 0x1600   // 气压数据地址
#define Y4000_REG_ATM_NUM 0x04      // 气压寄存器数量
#define Y4000_REG_BRUSH_ADDR 0x2F00 // 开启刷子地址
#define Y4000_REG_BRUSH_TIME 0x3000 // 刷子间隔时间地址

/**
 * @brief  读取Y4000数据（一次解析DO/PH/SAL三个参数）
 * @param  data: 输出参数，包含DO/PH/SAL三个浮点数
 * @return true:读取成功, false:读取失败
 * @note   一次Modbus读取，返回36字节数据，包含3个参数
 */
bool y4000_read(y4000_data_t *data) {
  uint8_t raw_data[40];
  uint16_t data_len;

  data->valid = false;

  /* 发送Modbus读取命令 */
  if (!modbus_read_holding(Y4000_ADDR, Y4000_REG_DPS_ADDR, Y4000_REG_DPS_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  /* 检查数据长度 */
  if (data_len < 36) {
    return false;
  }

  /* 解析三个浮点数 - 每个4字节，和裸机位置一致 */
  data->do_value = modbus_bytes_to_float(&raw_data[0]);   // DO值在第1-4字节
  data->ph_value = modbus_bytes_to_float(&raw_data[12]);  // PH值在第13-16字节
  data->sal_value = modbus_bytes_to_float(&raw_data[32]); // SAL值在第33-36字节
  data->valid = true;

  return true;
}

/**
 * @brief  读取气压值
 * @param  atm: 输出参数，气压值（kPa）
 * @return true:读取成功, false:读取失败
 */
bool y4000_read_atm(float *atm) {
  uint8_t raw_data[10];
  uint16_t data_len;

  if (!modbus_read_holding(Y4000_ADDR, Y4000_REG_ATM_ADDR, Y4000_REG_ATM_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  if (data_len >= 8) {
    /* 气压值在byte[4-7]位置 */
    *atm = modbus_bytes_to_float(&raw_data[4]);
    return true;
  }

  return false;
}

/**
 * @brief  开启刷子
 * @return true:成功, false:失败
 * @note   向寄存器0x2F00发送写命令
 */
bool y4000_open_brush(void) {
  return modbus_write_multiple(Y4000_ADDR, Y4000_REG_BRUSH_ADDR, 0, NULL, 0);
}

/**
 * @brief  获取刷子间隔时间
 * @param  interval: 输出参数，间隔时间（分钟）
 * @return true:成功, false:失败
 */
bool y4000_get_brush_time(uint16_t *interval) {
  uint8_t raw_data[10];
  uint16_t data_len;

  if (!modbus_read_holding(Y4000_ADDR, Y4000_REG_BRUSH_TIME, 0x01, raw_data,
                           &data_len)) {
    return false;
  }

  if (data_len >= 2) {
    /* 间隔时间：高字节*256 + 低字节 */
    *interval = (raw_data[1] << 8) | raw_data[0];
    return true;
  }

  return false;
}
