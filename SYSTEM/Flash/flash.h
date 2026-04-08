#ifndef __FLASH_H
#define __FLASH_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////
// Flash状态存储模块 - FreeRTOS版本 (STM32F407ZGT6)
//
// 功能说明：
//   1. 保存系统状态到内部Flash，实现断电恢复功能
//   2. 支持保存：传感器状态、通道状态、采集频率、上传频率
//   3. 添加FreeRTOS互斥锁保护和临界区保护
//
// Flash配置：
//   - MCU型号: STM32F407ZGT6 (1MB Flash)
//   - 存储扇区: Sector 11 (0x080E0000, 128KB)
//   - 擦除单位: 128KB (整个Sector)
//
// 注意事项：
//   - Flash擦除会阻塞任务（约几十ms）
//   - 避免频繁写入，Flash寿命约10万次擦写
//   - 建议仅在状态变化时保存
//
// 创建日期: 2026/01/04
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          Flash地址配置
 *===========================================================================*/

/* STM32F407ZGT6 Flash参数 */
#define FLASH_SIZE_KB 1024                   // Flash总大小(KB)
#define FLASH_SECTOR_SIZE_128KB (128 * 1024) // Sector大小(字节)

/* 状态存储区域配置 */
#define SYSTEM_STATE_SECTOR FLASH_Sector_11 // 使用Sector 11
#define SYSTEM_STATE_START_ADDR 0x080E0000  // Sector 11起始地址

/* 数据索引定义（通道状态不再持久化，索引0-2保留空位以兼容旧Flash布局） */
#define FLASH_IDX_ACQ_FREQUENCY 3  // 采集频率（保持原索引位置，向后兼容）
#define FLASH_IDX_UPL_FREQUENCY 4  // 上传频率

#define FLASH_DATA_COUNT 5    // 总共5个数据项（保持兼容旧布局）
#define FLASH_BUFFER_SIZE 200 // 缓冲区大小

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化Flash模块
 * @note   创建互斥锁，在FreeRTOS启动前调用
 */
void Flash_Init(void);

/**
 * @brief  保存系统状态到Flash
 * @note   会擦除整个Sector，操作耗时约几十ms
 *         使用互斥锁和临界区保护多任务访问
 */
void Flash_SaveSystemState(void);

/**
 * @brief  从Flash恢复系统状态
 * @note   在系统初始化时调用，恢复上次保存的状态
 */
void Flash_LoadSystemState(void);

/**
 * @brief  擦除系统状态Flash区域
 * @return 0:成功, -1:失败
 */
int Flash_EraseSystemState(void);

/*============================================================================
 *                          底层Flash操作函数
 *===========================================================================*/

/**
 * @brief  读取指定地址的半字(16位)
 * @param  address: Flash地址
 * @return 读取的16位数据
 */
uint16_t Flash_ReadHalfWord(uint32_t address);

/**
 * @brief  读取指定地址的字(32位)
 * @param  address: Flash地址
 * @return 读取的32位数据
 */
uint32_t Flash_ReadWord(uint32_t address);

/**
 * @brief  从指定地址读取多个半字
 * @param  startAddress: 起始地址
 * @param  readData: 数据缓冲区
 * @param  countToRead: 读取半字数量
 */
void Flash_ReadMoreData(uint32_t startAddress, uint16_t *readData,
                        uint16_t countToRead);

/**
 * @brief  向指定地址写入多个半字（含擦除）
 * @param  startAddress: 起始地址
 * @param  writeData: 数据缓冲区
 * @param  countToWrite: 写入半字数量
 * @note   会自动擦除对应的Sector
 */
void Flash_WriteMoreData(uint32_t startAddress, uint16_t *writeData,
                         uint16_t countToWrite);

#endif /* __FLASH_H */
