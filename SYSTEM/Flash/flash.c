#include "flash.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"
#include "task.h"
#include "usart.h"   // 用于调试输出
#include <stdbool.h> // 用于bool类型

//////////////////////////////////////////////////////////////////////////////////
// Flash状态存储模块 - FreeRTOS版本
//
// 硬件平台：STM32F407ZGT6
//   - Flash总容量：1MB (0x08000000 - 0x080FFFFF)
//   - 存储扇区：Sector 11 (0x080E0000 - 0x080FFFFF, 128KB)
//   - 擦除单位：整个扇区（128KB）
//   - 写入单位：半字（16位）或字（32位）
//
// 存储数据布局：
//   地址0x080E0000起，共5个uint16数据（10字节）：
//   [0] onechannel_state      - 通道1状态 (0=关闭, 1=开启)
//   [1] twochannel_state      - 通道2状态
//   [2] threechannel_state    - 通道3状态
//   [3] g_acquisition_frequency  - 采集频率（秒，范围1~3600）
//   [4] g_upload_frequency    - 上传频率（秒，范围10~3600）
//
// 功能特性：
//   1. 多任务保护：使用FreeRTOS互斥锁保护Flash读写操作
//   2. 数据有效性检查：检测0xFFFF（擦除状态），自动使用默认值
//   3. 原子性保证：使用临界区保护状态变量的批量读写
//   4. 自动保存：集成到Command模块，配置改变时自动触发
//
// 使用流程：
//   1. 系统启动时（main函数）：
//      Flash_Init();           // 创建互斥锁
//      Flash_LoadSystemState(); // 从Flash恢复配置
//
//   2. 运行时修改配置（Command模块自动调用）：
//      用户发送MQTT命令 → Command模块解析 → 修改内存变量
//      → Flash_SaveSystemState() → 保存到Flash
//
//   3. 系统重启后：
//      自动从Flash加载上次保存的配置
//
// 注意事项：
//   ⚠️ Flash擦写寿命约10万次，避免频繁写入
//   ⚠️ 擦写操作耗时约几十毫秒，会阻塞任务
//   ⚠️ Flash_LoadSystemState()在FreeRTOS启动前调用，不使用互斥锁
//   ⚠️ 写入过程中断电可能导致数据损坏
//
// 作者：蔡国华
// 创建日期：2026/01/04
// 最后更新：2026/01/11
// 版本：v1.1 - 添加数据有效性检查
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          外部变量引用
 *===========================================================================*/

/* 包含通道状态定义的中间层头文件 */
#include "WQInterface.h"

/* 频率配置 - 定义在各自模块 */
extern uint16_t g_acquisition_frequency; // 采集频率
extern uint16_t g_upload_frequency;      // 上传频率

/*============================================================================
 *                          私有变量
 *===========================================================================*/

/* FreeRTOS互斥锁 */
static SemaphoreHandle_t xFlashMutex = NULL;

/* Flash读写缓冲区（与裸机版本保持一致） */
static uint16_t wbuff[FLASH_BUFFER_SIZE];
static uint16_t rbuff[FLASH_BUFFER_SIZE];

/*============================================================================
 *                          底层Flash读写函数
 *===========================================================================*/

/**
 * @brief  读取指定地址的半字(16位)
 */
uint16_t Flash_ReadHalfWord(uint32_t address) {
  return *(__IO uint16_t *)address;
}

/**
 * @brief  读取指定地址的字(32位)
 */
uint32_t Flash_ReadWord(uint32_t address) {
  uint32_t temp1, temp2;
  temp1 = *(__IO uint16_t *)address;
  temp2 = *(__IO uint16_t *)(address + 2);
  return (temp2 << 16) + temp1;
}

/**
 * @brief  从指定地址读取多个半字
 */
void Flash_ReadMoreData(uint32_t startAddress, uint16_t *readData,
                        uint16_t countToRead) {
  uint16_t i;
  for (i = 0; i < countToRead; i++) {
    readData[i] = Flash_ReadHalfWord(startAddress + i * 2);
  }
}

/**
 * @brief  向指定地址写入多个半字（含擦除）
 * @note   STM32F407使用32位字编程，但我们仍保留半字接口兼容裸机版本
 */
void Flash_WriteMoreData(uint32_t startAddress, uint16_t *writeData,
                         uint16_t countToWrite) {
  uint16_t i;
  FLASH_Status status;

  /* 参数检查 */
  if (startAddress < FLASH_BASE || (startAddress + countToWrite * 2) >=
                                       (FLASH_BASE + FLASH_SIZE_KB * 1024)) {
    return; // 非法地址
  }

  /* 解锁Flash */
  FLASH_Unlock();

  /* 擦除对应的Sector（STM32F407根据地址自动确定扇区）*/
  status = FLASH_EraseSector(SYSTEM_STATE_SECTOR, VoltageRange_3);
  if (status != FLASH_COMPLETE) {
    FLASH_Lock();
    return;
  }

  /* 写入数据（使用半字编程） */
  for (i = 0; i < countToWrite; i++) {
    status = FLASH_ProgramHalfWord(startAddress + i * 2, writeData[i]);
    if (status != FLASH_COMPLETE) {
      FLASH_Lock();
      return;
    }
  }

  /* 上锁Flash */
  FLASH_Lock();
}

/*============================================================================
 *                          Flash高层API实现
 *===========================================================================*/

/**
 * @brief  初始化Flash模块
 */
void Flash_Init(void) {
  /* 创建互斥锁 */
  if (xFlashMutex == NULL) {
    xFlashMutex = xSemaphoreCreateMutex();
  }
}

/**
 * @brief  保存系统状态到Flash，在command文件中调用
 * @note   只保存通道状态和频率配置（共5个变量）
 */
void Flash_SaveSystemState(void) {
  uint16_t wlen;

  /* 获取互斥锁 */
  if (xFlashMutex == NULL ||
      xSemaphoreTake(xFlashMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  /* 进入临界区，读取所有状态变量（确保原子性） */
  taskENTER_CRITICAL();

  /* 只保存5个必要变量，将中间层当前探明的通道连接属性序列化 */
  wbuff[FLASH_IDX_CHANNEL1_STATE] = (uint16_t)WQInterface.Channel[0].connected;
  wbuff[FLASH_IDX_CHANNEL2_STATE] = (uint16_t)WQInterface.Channel[1].connected;
  wbuff[FLASH_IDX_CHANNEL3_STATE] = (uint16_t)WQInterface.Channel[2].connected;
  wbuff[FLASH_IDX_ACQ_FREQUENCY] = (uint16_t)g_acquisition_frequency;
  wbuff[FLASH_IDX_UPL_FREQUENCY] = (uint16_t)g_upload_frequency;

  taskEXIT_CRITICAL();

  /* 写入Flash（只写入5个数据） */
  wlen = FLASH_DATA_COUNT;
  Flash_WriteMoreData(SYSTEM_STATE_START_ADDR, wbuff, wlen);

  /* 释放互斥锁 */
  xSemaphoreGive(xFlashMutex);
}

/**
 * @brief  从Flash恢复系统状态
 * @note   只恢复通道状态和频率配置（共5个变量）
 *         如果Flash未初始化（数据为0xFFFF），则使用默认值
 */
void Flash_LoadSystemState(void) {
  uint16_t rlen;
  bool flash_valid = true;

  /* 读取Flash数据（只读取5个数据） */
  rlen = FLASH_DATA_COUNT;
  Flash_ReadMoreData(SYSTEM_STATE_START_ADDR, rbuff, rlen);

  /* 检查Flash数据有效性（判断是否首次使用或Flash被擦除） */
  if (rbuff[FLASH_IDX_ACQ_FREQUENCY] == 0xFFFF ||
      rbuff[FLASH_IDX_UPL_FREQUENCY] == 0xFFFF) {
    flash_valid = false;
  }

  /* 进入临界区，恢复所有状态变量 */
  taskENTER_CRITICAL();

  if (flash_valid) {
    /* 从Flash恢复状态至中间层 */
    WQInterface.Channel[0].connected = rbuff[FLASH_IDX_CHANNEL1_STATE];
    WQInterface.Channel[1].connected = rbuff[FLASH_IDX_CHANNEL2_STATE];
    WQInterface.Channel[2].connected = rbuff[FLASH_IDX_CHANNEL3_STATE];
    g_acquisition_frequency = rbuff[FLASH_IDX_ACQ_FREQUENCY];
    g_upload_frequency = rbuff[FLASH_IDX_UPL_FREQUENCY];
  } else {
    /* Flash未初始化，使用默认值关闭状态 */
    WQInterface.Channel[0].connected = 0;        
    WQInterface.Channel[1].connected = 0;        
    WQInterface.Channel[2].connected = 0;      
    g_acquisition_frequency = 5; // 默认5秒
    g_upload_frequency = 10;     // 默认10秒
  }

  taskEXIT_CRITICAL();
}

/**
 * @brief  擦除系统状态Flash区域
 */
int Flash_EraseSystemState(void) {
  FLASH_Status status;

  /* 获取互斥锁 */
  if (xFlashMutex == NULL ||
      xSemaphoreTake(xFlashMutex, portMAX_DELAY) != pdTRUE) {
    return -1;
  }

  /* 解锁Flash */
  FLASH_Unlock();

  /* 擦除Sector */
  status = FLASH_EraseSector(SYSTEM_STATE_SECTOR, VoltageRange_3);

  /* 上锁Flash */
  FLASH_Lock();

  /* 释放互斥锁 */
  xSemaphoreGive(xFlashMutex);

  if (status == FLASH_COMPLETE) {
    return 0;
  } else {
    return -1;
  }
}
