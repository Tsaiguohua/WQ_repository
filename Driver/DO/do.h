#ifndef DO_H
#define DO_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// DO（溶解氧）传感器驱动 - 简化版
//
// Modbus地址：0x05
// 数据寄存器：0x2600
//
// 使用方法：
//   float do_value;
//   if (do_read(&do_value)) {
//       printf("DO = %.2f mg/L\n", do_value);
//   }
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  读取溶解氧值
 * @param  do_value: 输出参数，DO值（mg/L）
 * @return true:读取成功, false:读取失败
 */
bool do_read(float *do_value);

#endif
