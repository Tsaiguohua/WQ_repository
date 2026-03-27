#ifndef __TF_H
#define __TF_H

#include "stm32f4xx.h"
#include <stdbool.h>

/**
 * @brief  初始化 TF 卡并挂载 FatFs 文件系统
 * @retval 0=成功，1=失败
 */
uint8_t TF_Hardware_Init(void);

/**
 * @brief  向指定文件末尾追加字符串内容（自动打开和关闭文件）
 * @param  filename: 文件名（包含路径，如 "0:Record.csv"）
 * @param  str: 要追加写入的字符串
 * @retval 0=成功，1=失败
 */
uint8_t TF_Append_String(const char* filename, const char* str);

#endif
