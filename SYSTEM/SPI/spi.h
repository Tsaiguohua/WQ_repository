/**
 ******************************************************************************
 * @file    spi.h
 * @author  STM32 Bootloader
 * @version V1.0
 * @date    2026-01-28
 * @brief   SPI1驱动头文件
 ******************************************************************************
 */

#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"

/* 板载W25Q128引脚定义（正点原子F407最小系统板） */
#define W25Q128_SPI SPI1
#define W25Q128_SPI_CLK RCC_APB2Periph_SPI1

#define W25Q128_SPI_GPIO_PORT GPIOB
#define W25Q128_SPI_GPIO_CLK RCC_AHB1Periph_GPIOB

#define W25Q128_SPI_SCK_PIN GPIO_Pin_3
#define W25Q128_SPI_SCK_PINSOURCE GPIO_PinSource3

#define W25Q128_SPI_MISO_PIN GPIO_Pin_4
#define W25Q128_SPI_MISO_PINSOURCE GPIO_PinSource4

#define W25Q128_SPI_MOSI_PIN GPIO_Pin_5
#define W25Q128_SPI_MOSI_PINSOURCE GPIO_PinSource5

#define W25Q128_CS_PIN GPIO_Pin_14
#define W25Q128_CS_PORT GPIOB

/* 函数声明 */
void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte(uint8_t ByteSend);


#endif
