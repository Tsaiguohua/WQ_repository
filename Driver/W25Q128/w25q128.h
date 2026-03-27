/**
 ******************************************************************************
 * @file    w25q128.h
 * @author  STM32 Bootloader
 * @version V1.0
 * @date    2026-01-28
 * @brief   W25Q128 Flash驱动头文件
 ******************************************************************************
 */

#ifndef __W25Q128_H
#define __W25Q128_H

#include "stm32f4xx.h"

/* W25Q128命令定义 */
#define W25Q128_CMD_WRITE_ENABLE 0x06       // 写使能
#define W25Q128_CMD_READ_STATUS_REG1 0x05   // 读状态寄存器1
#define W25Q128_CMD_READ_DATA 0x03          // 读数据
#define W25Q128_CMD_PAGE_PROGRAM 0x02       // 页编程
#define W25Q128_CMD_SECTOR_ERASE_4K 0x20    // 4KB扇区擦除
#define W25Q128_CMD_BLOCK_ERASE_64K 0xD8    // 64KB块擦除
#define W25Q128_CMD_READ_ID 0x9F            // 读取芯片ID

/* 函数声明 */
void W25Q128_Init(void);
void W25Q128_ReadID(uint8_t *MID, uint16_t *DID);
void W25Q128_WaitBusy(void);
void W25Q128_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
void W25Q128_PageProgram(uint32_t addr, uint8_t *buf, uint16_t len);
void W25Q128_SectorErase_4K(uint32_t sector_addr);
void W25Q128_Erase64KBlock_ByAddress(uint32_t addr);

#endif
