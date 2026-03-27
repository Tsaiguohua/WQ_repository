#include "cod.h"
#include "modbus.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// COD传感器驱动实现
//
// 功能说明：
//   COD传感器用于测量水体中的化学需氧量（COD）、总有机碳（TOC）和浊度（TUR）
//   同时可测量水温
//
// 数据解析位置（参考裸机版本 COD.c）：
//   TCT数据（0x2600）：
//   - 水温TEMP：byte[3-6]  (IEEE754浮点数)
//   - COD值：   byte[7-10]
//   - TOC值：   byte[11-14]
//
//   TUR数据（0x1200）：
//   - 浊度TUR： byte[3-6]
//
// Modbus寄存器（参考裸机版本）：
//   - 0x2600: 温度/COD/TOC数据（读7个寄存器）
//   - 0x1200: 浊度数据（读2个寄存器）
//   - 0x2500: 开始测量命令
//   - 0x2E00: 停止测量命令
//   - 0x3100: 开启刷子命令
//   - 0x3200: 刷子间隔时间
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

#define COD_ADDR 0x01 // Modbus从机地址（COD默认为0x01）

/* 寄存器地址 */
#define COD_REG_TCT_ADDR 0x2600   // 温度/COD/TOC数据地址
#define COD_REG_TCT_NUM 0x07      // 寄存器数量（7个=14字节）
#define COD_REG_TUR_ADDR 0x1200   // 浊度数据地址
#define COD_REG_TUR_NUM 0x02      // 寄存器数量（2个=4字节）
#define COD_REG_START_ADDR 0x2500 // 开始测量命令
#define COD_REG_STOP_ADDR 0x2E00  // 停止测量命令
#define COD_REG_BRUSH_ADDR 0x3100 // 开启刷子命令
#define COD_REG_BRUSH_TIME 0x3200 // 刷子间隔时间地址

/**
 * @brief  开启COD测量
 * @return true:成功, false:失败
 * @note   对应裸机的 COD_Open() 函数
 */
bool cod_open(void) {
  uint8_t dummy[4];
  uint16_t data_len;
  return modbus_read_holding(COD_ADDR, COD_REG_START_ADDR, 0x01, dummy,
                             &data_len);
}

/**
 * @brief  关闭COD测量
 * @return true:成功, false:失败
 * @note   对应裸机的 COD_Close() 函数
 */
bool cod_close(void) {
  uint8_t dummy[4];
  uint16_t data_len;
  return modbus_read_holding(COD_ADDR, COD_REG_STOP_ADDR, 0x01, dummy,
                             &data_len);
}

/**
 * @brief  读取温度/COD/TOC数据
 * @param  data: 输出参数，包含temp/cod/toc字段
 * @return true:读取成功, false:读取失败
 * @note   对应裸机的 COD_GetTCT() 函数
 */
bool cod_read_tct(cod_data_t *data) {
  uint8_t raw_data[20];
  uint16_t data_len;

  /* 发送Modbus读取命令：读取0x2600寄存器，7个寄存器 */
  if (!modbus_read_holding(COD_ADDR, COD_REG_TCT_ADDR, COD_REG_TCT_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  /* 检查数据长度，应至少为14字节 */
  if (data_len >= 14) {
    /* 解析浮点数（和裸机一致的位置） */
    data->temp = modbus_bytes_to_float(&raw_data[0]); // byte[0-3]
    data->cod = modbus_bytes_to_float(&raw_data[4]);  // byte[4-7]
    data->toc = modbus_bytes_to_float(&raw_data[8]);  // byte[8-11]
    data->valid = true;
    return true;
  }

  return false;
}

/**
 * @brief  读取浊度数据
 * @param  tur: 输出参数，浊度值
 * @return true:读取成功, false:读取失败
 * @note   对应裸机的 COD_GetTUR() 函数
 */
bool cod_read_tur(float *tur) {
  uint8_t raw_data[10];
  uint16_t data_len;

  /* 发送Modbus读取命令：读取0x1200寄存器，2个寄存器 */
  if (!modbus_read_holding(COD_ADDR, COD_REG_TUR_ADDR, COD_REG_TUR_NUM,
                           raw_data, &data_len)) {
    return false;
  }

  /* 检查数据长度，应至少为4字节 */
  if (data_len >= 4) {
    *tur = modbus_bytes_to_float(&raw_data[0]);
    return true;
  }

  return false;
}

/**
 * @brief  读取所有COD数据（封装了读取TCT和TUR的流程）
 * @param  data: 输出参数，包含temp/cod/toc/tur字段
 * @return true:读取成功, false:读取失败
 * @note   为了兼容acquisition.c中的统一接口而添加
 */
bool cod_read(cod_data_t *data) {
  if (data == NULL) {
    return false;
  }

  data->valid = false;

  /* 1. 读取温度/COD/TOC */
  if (!cod_read_tct(data)) {
    return false;
  }

  /* 2. 读取浊度 */
  if (!cod_read_tur(&data->tur)) {
    /* 浊度读取失败，但TCT数据仍有效 */
    data->tur = 0.0f;
  }

  data->valid = true;
  return true;
}

/**
 * @brief  开启清洁刷子
 * @return true:成功, false:失败
 * @note   对应裸机的 COD_OpenBrush() 函数
 */
bool cod_open_brush(void) {
  return modbus_write_multiple(COD_ADDR, COD_REG_BRUSH_ADDR, 0, NULL, 0);
}

/**
 * @brief  获取刷子间隔时间
 * @param  interval: 输出参数，间隔时间（分钟）
 * @return true:成功, false:失败
 * @note   对应裸机的 COD_GetBrushTime() 函数
 */
bool cod_get_brush_time(uint16_t *interval) {
  uint8_t raw_data[10];
  uint16_t data_len;

  if (!modbus_read_holding(COD_ADDR, COD_REG_BRUSH_TIME, 0x01, raw_data,
                           &data_len)) {
    return false;
  }

  if (data_len >= 2) {
    /* 间隔时间：低字节在前，高字节在后（和裸机一致） */
    *interval = (raw_data[1] << 8) | raw_data[0];
    return true;
  }

  return false;
}

/**
 * @brief  设置刷子间隔时间
 * @param  interval: 间隔时间（分钟）
 * @return true:成功, false:失败
 * @note   对应裸机的 COD_SetBrushTime() 函数
 */
bool cod_set_brush_time(uint16_t interval) {
  uint8_t data[2];
  data[0] = interval & 0xFF;
  data[1] = (interval >> 8) & 0xFF;

  return modbus_write_multiple(COD_ADDR, COD_REG_BRUSH_TIME, 0x01, data, 2);
}
