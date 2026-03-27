#include "self_exam.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "task.h"
#include "rs485.h"
#include <stdio.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// 传感器自检模块 - 纯硬件驱动层 (重构简化版)
//
// 功能说明：
//   本模块负责系统启动时的底层物理控制。
//   1. 使用推挽输出操作 PE0/PE1/PE2 脚来闭合/断开高侧开关（为通道上电）。
//   2. 组装并发送 0xFF 广播 Modbus 寻址帧，获取并返回传感器地址报文。
//   纯净的驱动实现：【不保存】任务相关的全局变量，【不负责】类型分配和系统注册。
//
// 硬件接口 (高侧开关替代继电器)：
//   芯片型号：TI TPS1H200AQDGNRQ1
//   控制逻辑：IN 引脚高电平导通 (PE0/PE1/PE2分别控制1/2/3通道)
//   配线说明：
//   - IN 串联 1k 欧姆电阻接入 PE0 / PE1 / PE2
//   - DIAGEN 接地，FAULT 悬空，CL 接地，DELAY 接地
//   - VS 并联电容接 12V，OUT 接传感器通道正极
//////////////////////////////////////////////////////////////////////////////////

/* 通道1控制引脚: PE0 */
#define CHANNEL1_GPIO_PORT GPIOE
#define CHANNEL1_GPIO_PIN GPIO_Pin_0
#define CHANNEL1_GPIO_CLK RCC_AHB1Periph_GPIOE

/* 通道2控制引脚: PE1 */
#define CHANNEL2_GPIO_PORT GPIOE
#define CHANNEL2_GPIO_PIN GPIO_Pin_1
#define CHANNEL2_GPIO_CLK RCC_AHB1Periph_GPIOE

/* 通道3控制引脚: PE2 */
#define CHANNEL3_GPIO_PORT GPIOE
#define CHANNEL3_GPIO_PIN GPIO_Pin_2
#define CHANNEL3_GPIO_CLK RCC_AHB1Periph_GPIOE

/* 通道控制宏 */
#define CHANNEL1_ON() GPIO_SetBits(CHANNEL1_GPIO_PORT, CHANNEL1_GPIO_PIN)
#define CHANNEL1_OFF() GPIO_ResetBits(CHANNEL1_GPIO_PORT, CHANNEL1_GPIO_PIN)
#define CHANNEL2_ON() GPIO_SetBits(CHANNEL2_GPIO_PORT, CHANNEL2_GPIO_PIN)
#define CHANNEL2_OFF() GPIO_ResetBits(CHANNEL2_GPIO_PORT, CHANNEL2_GPIO_PIN)
#define CHANNEL3_ON() GPIO_SetBits(CHANNEL3_GPIO_PORT, CHANNEL3_GPIO_PIN)
#define CHANNEL3_OFF() GPIO_ResetBits(CHANNEL3_GPIO_PORT, CHANNEL3_GPIO_PIN)

/*============================================================================
 *                          CRC16计算（和裸机一致）
 *===========================================================================*/

static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601,
    0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0,
    0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81,
    0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0,
    0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081,
    0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00,
    0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700,
    0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
    0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01,
    0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1,
    0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101,
    0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481,
    0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
    0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581,
    0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040};

static uint16_t CRC_Compute(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF];
  }
  return crc;
}

/*============================================================================
 *                          GPIO初始化与操作
 *===========================================================================*/

void SelfExam_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* 使能GPIO时钟 */
  RCC_AHB1PeriphClockCmd(CHANNEL1_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(CHANNEL2_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(CHANNEL3_GPIO_CLK, ENABLE);

  /* 配置通道控制引脚模式（推挽输出） */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* 通道1 */
  GPIO_InitStructure.GPIO_Pin = CHANNEL1_GPIO_PIN;
  GPIO_Init(CHANNEL1_GPIO_PORT, &GPIO_InitStructure);

  /* 通道2 */
  GPIO_InitStructure.GPIO_Pin = CHANNEL2_GPIO_PIN;
  GPIO_Init(CHANNEL2_GPIO_PORT, &GPIO_InitStructure);

  /* 通道3 */
  GPIO_InitStructure.GPIO_Pin = CHANNEL3_GPIO_PIN;
  GPIO_Init(CHANNEL3_GPIO_PORT, &GPIO_InitStructure);

  /* 系统上电初期主动掐断所有通道路，确保冷启动状态 */
  SelfExam_CloseAllChannels();
}

void SelfExam_OpenChannel(uint8_t channel) {
  switch (channel) {
    case 1: CHANNEL1_ON(); break;
    case 2: CHANNEL2_ON(); break;
    case 3: CHANNEL3_ON(); break;
    default: break;
  }
}

void SelfExam_CloseChannel(uint8_t channel) {
  switch (channel) {
    case 1: CHANNEL1_OFF(); break;
    case 2: CHANNEL2_OFF(); break;
    case 3: CHANNEL3_OFF(); break;
    default: break;
  }
}

void SelfExam_CloseAllChannels(void) {
  CHANNEL1_OFF();
  CHANNEL2_OFF();
  CHANNEL3_OFF();
}


#include "self_exam.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "rs485.h" // 🌟 包含 RS485 的头文件
#include <stdio.h>
#include <string.h>

/*============================================================================
 * 探测核心方法 (🌟 重构后使用高级API)
 *===========================================================================*/

uint8_t SelfExam_ProbeAddress(void) {
  uint8_t tx_buf[8];
  uint8_t rx_buf[64]; // 自己准备一个本地的盆来接数据
  uint16_t rx_len = 0;
  uint16_t crc;
  uint8_t address = 0;

  /* 组装广播查询帧: 0xFF 0x03 0x30 0x00 0x00 0x01 [CRC_L] [CRC_H] */
  tx_buf[0] = 0xFF; 
  tx_buf[1] = 0x03; 
  tx_buf[2] = 0x30; 
  tx_buf[3] = 0x00; 
  tx_buf[4] = 0x00; 
  tx_buf[5] = 0x01; 
  crc = CRC_Compute(tx_buf, 6);
  tx_buf[6] = crc & 0xFF;        
  tx_buf[7] = (crc >> 8) & 0xFF; 

  /* 🌟 使用 rs485.c 提供的API：自动加锁、自动发送、自动等待接收(超时500ms) */
  if (rs485_transfer(tx_buf, 8, rx_buf, &rx_len, 500)) {
      
      /* 如果成功收到了响应 (且长度大于等于 7 字节) */
      if (rx_len >= 7) {
          uint16_t recv_crc = rx_buf[rx_len - 2] | (rx_buf[rx_len - 1] << 8);
          uint16_t calc_crc = CRC_Compute(rx_buf, rx_len - 2);
          
          // Y4000 等特殊传感器的设备不遵循严谨的 Modbus CRC
          if (calc_crc == recv_crc || (rx_buf[0] == 0x07 && rx_buf[1] == 0x03)) {
              address = rx_buf[0];
          } else {
             // 特殊判断兜底
             if (rx_buf[0] == 0x07 && rx_buf[1] == 0x03) {
                 address = 0x07;
             }
          }
      }
  }

  return address;
}

