/**
 ******************************************************************************
 * @file    w25q128.c
 * @author  STM32 Bootloader
 * @version V1.0
 * @date    2026-01-28
 * @brief   W25Q128 Flash驱动实现(16MB SPI Flash)
 ******************************************************************************
 * @description
 * 本文件实现了W25Q128 Flash芯片的基本操作，包括：
 * 1. 初始化并读取芯片ID
 * 2. 数据读取（支持任意地址、任意长度）
 * 3. 页编程（256字节对齐）
 * 4. 扇区擦除（4KB）
 * 5. 块擦除（64KB）
 *
 * 硬件连接（正点原子F407板载W25Q128）：
 * - SPI1_SCK:  PB3
 * - SPI1_MISO: PB4
 * - SPI1_MOSI: PB5
 * - CS:        PB14 (软件控制)
 *
 * 芯片参数：
 * - 容量: 16MB (128Mbit)
 * - 页大小: 256字节
 * - 扇区大小: 4KB
 * - 块大小: 64KB
 * - 最高SPI时钟: 104MHz（本驱动配置21MHz）
 ******************************************************************************
 */

#include "w25q128.h"
#include "spi.h"
#include <stdio.h>

/**
 * @brief  W25Q128初始化
 * @param  None
 * @retval None
 * @note   初始化SPI接口并读取芯片ID
 *         正常ID: MID=0xEF, DID=0x4018 (W25Q128)
 */
void W25Q128_Init(void) {

  MySPI_Init(); // 初始化SPI1接口
}



/**
 * @brief  等待W25Q128空闲（忙状态结束）
 * @param  None
 * @retval None
 * @note   W25Q128在执行擦除/编程操作时BUSY位=1
 *         必须等待BUSY=0才能进行下一步操作
 *         典型擦除时间：4KB扇区约45ms，64KB块约150ms
 */
void W25Q128_WaitBusy(void) {
  uint8_t status;
  do {
    MySPI_Start();
    MySPI_SwapByte(W25Q128_CMD_READ_STATUS_REG1); // 读状态寄存器1 (0x05)
    status = MySPI_SwapByte(0xFF);
    MySPI_Stop();
  } while (status & 0x01); // BUSY位在bit0，等待变为0
}

/**
 * @brief  发送写使能命令
 * @param  None
 * @retval None
 * @note   W25Q128在执行编程/擦除前必须先使能写操作
 *         写使能后BUSY位会自动置1，操作完成后自动清零
 */
static void W25Q128_WriteEnable(void) {
  MySPI_Start();
  MySPI_SwapByte(W25Q128_CMD_WRITE_ENABLE); // 写使能命令(0x06)
  MySPI_Stop();
}

/**
 * @brief  读取数据
 * @param  addr: 起始地址（0x000000 - 0xFFFFFF）
 * @param  buf: 数据缓冲区指针
 * @param  len: 要读取的字节数
 * @retval None
 * @note   支持跨页、跨扇区读取
 *         读取速度典型值：50MB/s @ 104MHz SPI
 */
void W25Q128_ReadData(uint32_t addr, uint8_t *buf, uint32_t len) {
  MySPI_Start();
  MySPI_SwapByte(W25Q128_CMD_READ_DATA); // 读数据命令(0x03)
  MySPI_SwapByte((addr >> 16) & 0xFF);   // 地址高字节
  MySPI_SwapByte((addr >> 8) & 0xFF);    // 地址中字节
  MySPI_SwapByte(addr & 0xFF);           // 地址低字节

  for (uint32_t i = 0; i < len; i++) {
    buf[i] = MySPI_SwapByte(0xFF); // 读取数据
  }
  MySPI_Stop();
}

/**
 * @brief  页编程（最多256字节）
 * @param  addr: 起始地址（页对齐，页大小256字节）
 * @param  buf: 数据缓冲区指针
 * @param  len: 要写入的字节数（最多256）
 * @retval None
 * @note   1. 页编程前必须确保该页已擦除（全0xFF）
 *         2. 如果addr不是页对齐，写入会自动回卷到页首
 *         3. 典型编程时间：256字节约0.7ms
 */
void W25Q128_PageProgram(uint32_t addr, uint8_t *buf, uint16_t len) {
  if (len > 256)
    len = 256; // 限制最大长度

  W25Q128_WriteEnable(); // 写使能

  MySPI_Start();
  MySPI_SwapByte(W25Q128_CMD_PAGE_PROGRAM); // 页编程命令(0x02)
  MySPI_SwapByte((addr >> 16) & 0xFF);
  MySPI_SwapByte((addr >> 8) & 0xFF);
  MySPI_SwapByte(addr & 0xFF);

  for (uint16_t i = 0; i < len; i++) {
    MySPI_SwapByte(buf[i]);
  }
  MySPI_Stop();

  W25Q128_WaitBusy(); // 等待编程完成
}

/**
 * @brief  擦除4KB扇区
 * @param  sector_addr: 扇区起始地址（必须4KB对齐，如0x0000, 0x1000, 0x2000...）
 * @retval None
 * @note   擦除后该扇区所有字节变为0xFF
 *         典型擦除时间：45ms
 *         擦除次数：最少10万次
 */
void W25Q128_SectorErase_4K(uint32_t sector_addr) {
  W25Q128_WriteEnable();

  MySPI_Start();
  MySPI_SwapByte(W25Q128_CMD_SECTOR_ERASE_4K); // 扇区擦除命令(0x20)
  MySPI_SwapByte((sector_addr >> 16) & 0xFF);
  MySPI_SwapByte((sector_addr >> 8) & 0xFF);
  MySPI_SwapByte(sector_addr & 0xFF);
  MySPI_Stop();

  W25Q128_WaitBusy(); // 等待擦除完成（约45ms）
}

/**
 * @brief  擦除64KB块
 * @param  addr: 块地址（必须64KB对齐，如0x00000, 0x10000, 0x20000...）
 * @retval None
 * @note   擦除后该块所有字节变为0xFF
 *         典型擦除时间：150ms
 *         比扇区擦除更快（擦除大区域时推荐使用）
 */
void W25Q128_Erase64KBlock_ByAddress(uint32_t addr) {
  W25Q128_WriteEnable();

  MySPI_Start();
  MySPI_SwapByte(W25Q128_CMD_BLOCK_ERASE_64K); // 块擦除命令(0xD8)
  MySPI_SwapByte((addr >> 16) & 0xFF);
  MySPI_SwapByte((addr >> 8) & 0xFF);
  MySPI_SwapByte(addr & 0xFF);
  MySPI_Stop();

  W25Q128_WaitBusy(); // 等待擦除完成（约150ms）
}
