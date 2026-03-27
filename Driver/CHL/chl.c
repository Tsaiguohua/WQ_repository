#include "chl.h"
#include "modbus.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// CHL（叶绿素）传感器驱动实现
//
// 功能说明：
//   CHL传感器用于测量水体中叶绿素含量
//   叶绿素是评估水体富营养化程度的重要指标
//
// 数据解析位置：
//   CHL数据（0x2600）：
//   - CHL值： byte[4-7]  (第5-8字节)
//
// Modbus寄存器：
//   - 0x2600: CHL数据地址
//   - 0x2500: 开始测量命令
//   - 0x2E00: 停止测量命令
//   - 0x3100: 开启刷子命令
//   - 0x3000: 刷子间隔时间（可读写）
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define CHL_ADDR 0x03 // Modbus从机地址

#define CHL_REG_DATA_ADDR 0x2600  // CHL数据地址
#define CHL_REG_DATA_NUM 0x07     // 寄存器数量
#define CHL_REG_START_ADDR 0x2500 // 开始测量
#define CHL_REG_STOP_ADDR 0x2E00  // 停止测量
#define CHL_REG_BRUSH_ADDR 0x3100 // 开启刷子
#define CHL_REG_BRUSH_TIME 0x3000 // 刷子间隔时间

/**
 * @brief  开启CHL测量
 * @return true:成功, false:失败
 * @note   开启后需要等待约1秒让传感器稳定
 */
bool chl_open(void) {
  uint8_t dummy[4];
  uint16_t data_len;
  return modbus_read_holding(CHL_ADDR, CHL_REG_START_ADDR, 0x01, dummy,
                             &data_len);
}

/**
 * @brief  关闭CHL测量
 * @return true:成功, false:失败
 * @note   读取完毕后关闭可以省电
 */
bool chl_close(void) {
  uint8_t dummy[4];
  uint16_t data_len;
  return modbus_read_holding(CHL_ADDR, CHL_REG_STOP_ADDR, 0x01, dummy,
                             &data_len);
}

/**
 * @brief  读取CHL值
 * @param  chl_value: 输出参数，叶绿素值
 * @return true:读取成功, false:读取失败
 */
bool chl_read(float *chl_value) {
  uint8_t raw_data[20];
  uint16_t data_len;

  /* 发送Modbus读取命令 */
  if (!modbus_read_holding(CHL_ADDR, CHL_REG_DATA_ADDR, CHL_REG_DATA_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  if (data_len >= 8) {
    /* CHL值在byte[4-7]位置 */
    *chl_value = modbus_bytes_to_float(&raw_data[4]);
    return true;
  }

  return false;
}

/**
 * @brief  开启清洁刷子
 * @return true:成功, false:失败
 */
bool chl_open_brush(void) {
  return modbus_write_multiple(CHL_ADDR, CHL_REG_BRUSH_ADDR, 0, NULL, 0);
}

/**
 * @brief  获取刷子间隔时间
 * @param  interval: 输出参数，间隔时间（分钟）
 * @return true:成功, false:失败
 */
bool chl_get_brush_time(uint16_t *interval) {
  uint8_t raw_data[10];
  uint16_t data_len;

  if (!modbus_read_holding(CHL_ADDR, CHL_REG_BRUSH_TIME, 0x01, raw_data,
                           &data_len)) {
    return false;
  }

  if (data_len >= 2) {
    *interval = (raw_data[1] << 8) | raw_data[0];
    return true;
  }

  return false;
}

/**
 * @brief  设置刷子间隔时间
 * @param  interval: 间隔时间（分钟）
 * @return true:成功, false:失败
 */
bool chl_set_brush_time(uint16_t interval) {
  uint8_t data[2];
  data[0] = interval & 0xFF;
  data[1] = (interval >> 8) & 0xFF;

  return modbus_write_multiple(CHL_ADDR, CHL_REG_BRUSH_TIME, 0x01, data, 2);
}
