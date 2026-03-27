#ifndef COD_H
#define COD_H

#include "../sensor_common.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// COD传感器驱动 - 简化版
//
// Modbus地址：0x01
// 特点：需要先开启测量，再读取数据，最后关闭
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  开启COD测量
 */
bool cod_open(void);

/**
 * @brief  关闭COD测量
 */
bool cod_close(void);

/**
 * @brief  读取温度/COD/TOC数据
 */
bool cod_read_tct(cod_data_t *data);

/**
 * @brief  读取所有COD数据（封装了open/read/close流程）
 * @param  data: 输出参数
 * @return true:读取成功
 */
bool cod_read(cod_data_t *data);

/**
 * @brief  读取浊度数据
 */
bool cod_read_tur(float *tur);

/**
 * @brief  开启清洁刷子
 */
bool cod_open_brush(void);

/**
 * @brief  获取刷子间隔时间
 * @param  interval: 输出参数，间隔时间（分钟）
 */
bool cod_get_brush_time(uint16_t *interval);

/**
 * @brief  设置刷子间隔时间
 * @param  interval: 间隔时间（分钟）
 */
bool cod_set_brush_time(uint16_t interval);

#endif
