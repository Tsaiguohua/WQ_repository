/**
 ******************************************************************************
 * @file    spi.c
 * @author  STM32 Bootloader
 * @version V1.0
 * @date    2026-01-28
 * @brief   SPI1驱动实现（用于W25Q128通信）
 ******************************************************************************
 * @description
 * 本文件实现了SPI1的初始化和通信函数，专用于与板载W25Q128通信
 * 
 * 硬件连接（正点原子F407最小系统板）：
 * - SPI1_SCK:  PB3  (复用功能AF5)
 * - SPI1_MISO: PB4  (复用功能AF5)
 * - SPI1_MOSI: PB5  (复用功能AF5)
 * - CS:        PB14 (软件GPIO控制)
 * 
 * SPI配置：
 * - 模式：主机模式
 * - 时钟极性：CPOL=0（空闲时SCK为低电平）
 * - 时钟相位：CPHA=0（第一个边沿采样）
 * - 数据位：8位
 * - 波特率：84MHz/4 = 21MHz
 * - CS控制：软件控制
 ******************************************************************************
 */

#include "spi.h"

/**
 * @brief  SPI1初始化（配置GPIO和SPI外设）
 * @param  None
 * @retval None
 * @note   SPI1时钟来自APB2（84MHz）
 *         分频后SPI时钟为21MHz，满足W25Q128要求（最高104MHz）
 */
void MySPI_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  /* 使能时钟 */
  RCC_AHB1PeriphClockCmd(W25Q128_SPI_GPIO_CLK, ENABLE); // GPIOB时钟
  RCC_APB2PeriphClockCmd(W25Q128_SPI_CLK, ENABLE);      // SPI1时钟

  /* 配置GPIO复用功能为SPI1 */
  GPIO_PinAFConfig(W25Q128_SPI_GPIO_PORT, W25Q128_SPI_SCK_PINSOURCE,
                   GPIO_AF_SPI1);
  GPIO_PinAFConfig(W25Q128_SPI_GPIO_PORT, W25Q128_SPI_MISO_PINSOURCE,
                   GPIO_AF_SPI1);
  GPIO_PinAFConfig(W25Q128_SPI_GPIO_PORT, W25Q128_SPI_MOSI_PINSOURCE,
                   GPIO_AF_SPI1);

  /* 配置SPI引脚：SCK、MOSI、MISO */
  GPIO_InitStructure.GPIO_Pin =
      W25Q128_SPI_SCK_PIN | W25Q128_SPI_MOSI_PIN | W25Q128_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     // 高速
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // 无上下拉
  GPIO_Init(W25Q128_SPI_GPIO_PORT, &GPIO_InitStructure);

  /* 配置片选引脚CS（软件控制，普通GPIO输出） */
  GPIO_InitStructure.GPIO_Pin = W25Q128_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // 默认上拉（CS空闲为高）
  GPIO_Init(W25Q128_CS_PORT, &GPIO_InitStructure);

  /* 配置SPI1参数 */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       // 主机模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   // 8位数据
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                          // 时钟极性：空闲低电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                        // 时钟相位：第一个边沿采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           // 软件NSS管理
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  // 84MHz/4 = 21MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  // MSB先行
  SPI_InitStructure.SPI_CRCPolynomial = 7;                            // CRC多项式（未使用）
  SPI_Init(W25Q128_SPI, &SPI_InitStructure);

  /* 使能SPI1 */
  SPI_Cmd(W25Q128_SPI, ENABLE);

  /* CS默认拉高（未选中） */
  GPIO_SetBits(W25Q128_CS_PORT, W25Q128_CS_PIN);
}

/**
 * @brief  开始SPI通信（拉低CS）
 * @param  None
 * @retval None
 */
void MySPI_Start(void) { 
  GPIO_ResetBits(W25Q128_CS_PORT, W25Q128_CS_PIN); 
}

/**
 * @brief  停止SPI通信（拉高CS）
 * @param  None
 * @retval None
 */
void MySPI_Stop(void) { 
  GPIO_SetBits(W25Q128_CS_PORT, W25Q128_CS_PIN); 
}

/**
 * @brief  SPI交换一个字节（发送并接收）
 * @param  ByteSend: 要发送的字节
 * @retval uint8_t: 接收到的字节
 * @note   SPI是全双工通信，发送的同时会接收数据
 */
uint8_t MySPI_SwapByte(uint8_t ByteSend) {
  /* 等待发送缓冲区空 */
  while (SPI_I2S_GetFlagStatus(W25Q128_SPI, SPI_I2S_FLAG_TXE) == RESET)
    ;
  
  /* 发送数据 */
  SPI_I2S_SendData(W25Q128_SPI, ByteSend);
  
  /* 等待接收缓冲区非空 */
  while (SPI_I2S_GetFlagStatus(W25Q128_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    ;
  
  /* 返回接收到的数据 */
  return SPI_I2S_ReceiveData(W25Q128_SPI);
}
