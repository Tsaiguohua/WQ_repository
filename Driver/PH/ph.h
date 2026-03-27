#ifndef PH_H
#define PH_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// PH传感器驱动 - 简化版
//
// Modbus地址：0x04
// 数据寄存器：0x2800
//
// 使用方法：
//   float ph_value;
//   if (ph_read(&ph_value)) {
//       printf("PH = %.2f\n", ph_value);
//   }
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  读取PH值
 * @param  ph_value: 输出参数，PH值（0-14）
 * @return true:读取成功, false:读取失败
 */
bool ph_read(float *ph_value);

#endif
