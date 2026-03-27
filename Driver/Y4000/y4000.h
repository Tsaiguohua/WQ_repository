#ifndef Y4000_H
#define Y4000_H

#include "../sensor_common.h"
#include <stdbool.h>
#include <stdint.h>


//////////////////////////////////////////////////////////////////////////////////
// Y4000多参数传感器驱动 - 简化版
//
// Modbus地址：0x07
// 功能：一次读取PH/DO/SAL三个参数
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  读取Y4000数据（一次解析PH/DO/SAL三个参数）
 */
bool y4000_read(y4000_data_t *data);

/**
 * @brief  读取Y4000气压值
 */
bool y4000_read_atm(float *atm);

/**
 * @brief  开启Y4000刷子
 */
bool y4000_open_brush(void);

/**
 * @brief  获取Y4000刷子间隔时间
 * @param  interval: 输出参数，间隔时间（分钟）
 */
bool y4000_get_brush_time(uint16_t *interval);

#endif
