#ifndef SAL_H
#define SAL_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// SAL（盐度）传感器驱动 - 简化版
//
// Modbus地址：0x06
// 数据寄存器：0x2602
//
// 使用方法：
//   float sal_value;
//   if (sal_read(&sal_value)) {
//       printf("SAL = %.2f ppt\n", sal_value);
//   }
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  读取盐度值
 * @param  sal_value: 输出参数，盐度值（ppt）
 * @return true:读取成功, false:读取失败
 */
bool sal_read(float *sal_value);

#endif
