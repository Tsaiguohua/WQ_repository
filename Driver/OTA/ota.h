#ifndef __OTA_H
#define __OTA_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdbool.h>

//////////////////////////////////////////////////////////////////////////////////
// OTA固件升级模块 - FreeRTOS版本
//
// 功能说明：
//   1. 基于W25Q128外部Flash实现空中固件升级（Over-The-Air Update）
//   2. 通过MQTT接收服务器下发的固件数据包（Base64编码，1KB/包）
//   3. 支持状态机管理：空闲、检查版本、擦除Flash、下载固件、验证
//   4. OTA期间自动暂停所有非关键任务（采集、上传、APP、OLED、心跳、看门狗）
//   5. 支持下载超时检测（60秒）和失败后自动恢复任务运行
//   6. 升级成功后写入标志位，重启由Bootloader完成固件烧录和跳转
//
// 硬件配置：
//   - Flash芯片: W25Q128 (16MB, SPI接口)
//   - SPI引脚: 由w25q128.c定义（通常为SPI1或SPI2）
//   - 4G模块: UART4透传模式，接收MQTT下发的OTA指令
//
// W25Q128分区规划（与Bootloader一致）：
//   ┌─────────────────────────────────────────────────────┐
//   │ 0x000000-0x000FFF (4KB)  : OTA标志区                │
//   │   - 0x000000: OTA升级标志 (0xAA55AA55=需要升级)    │
//   │   - 0x000004: 固件长度 (字节数)                     │
//   │   - 0x000008: 启动计数器 (验证新固件用)            │
//   ├─────────────────────────────────────────────────────┤
//   │ 0x001000-0x0FFFFF (1020KB): OTA下载区（新固件）    │
//   ├─────────────────────────────────────────────────────┤
//   │ 0x100000-0x1FFFFF (1MB)   : APP备份区(预留)        │
//   └─────────────────────────────────────────────────────┘
//
// MQTT指令格式：
//   ▶ 查询版本: "config,upin,1.0"
//     回复: "config,upin,ok,<当前版本号>"
//
//   ▶ 开始下载: "config,upin,ok"
//     - APP立即暂停所有非关键任务
//     - 开始擦除Flash下载区
//     - 回复: "upfile,ready"
//
//   ▶ 数据包: "upfile,data,<Base64数据>,<包序号>"
//     - 每包最大1024字节（Base64解码后）
//     - 按序号写入Flash
//     - 实时更新下载进度
//
//   ▶ 下载完成: "upfile,end,<总字节数>"
//     - 验证下载大小
//     - 写入OTA标志和固件长度
//     - 回复: "upfile,ok,success"
//     - 3秒后自动重启，由Bootloader接管
//
// 状态机流程：
//   IDLE → CHECKING (查询版本) → ERASING (擦除Flash)
//     → DOWNLOADING (接收数据包) → COMPLETE (写标志、重启)
//     → 任何阶段超时60秒 → IDLE (恢复所有任务)
//
// 与Bootloader配合：
//   1. APP写入OTA标志(0xAA55AA55)和固件长度到Flash
//   2. APP调用NVIC_SystemReset()重启
//   3. Bootloader检测到OTA标志，开始固件搬运：
//      - 从W25Q128读取新固件
//      - 写入STM32内部Flash的APP区(0x08010000起)
//      - 验证完成后清除OTA标志
//      - 跳转到新APP
//
// 安全机制：
//   ✓ 多任务暂停：防止UART4/SPI总线冲突
//   ✓ 看门狗暂停：防止下载时间过长触发系统复位
//   ✓ 超时保护：60秒无数据自动退出OTA，恢复正常运行
//   ✓ CRC校验：可选支持固件CRC验证（需扩展）
//   ✓ 分片擦除：每次擦除64KB，避免长时间阻塞（预留设计）
//
// 注意事项：
//   ⚠️ OTA升级时系统会暂停数据采集和上传，建议在低负载时段执行
//   ⚠️ 必须确保W25Q128与Bootloader分区定义完全一致
//   ⚠️ 固件大小不能超过1020KB（0x001000-0x0FFFFF区域）
//   ⚠️ 升级期间断电会导致Flash数据不完整，Bootloader应有回滚机制
//
// 创建日期: 2026/01/28
// 作者: 蔡国华
// 最后更新: 2026/02/02 (新增多任务安全暂停机制)
//////////////////////////////////////////////////////////////////////////////////

// ==================== W25Q128 OTA存储分区定义（双APP版本）====================
// ==================== 必须与bootloader.h中的定义完全一致！

// ========== 元数据区 (4KB) ==========
#define OTA_FLAG_ADDRESS 0x000000          // OTA升级标志（4字节）
#define OTA_FW_LENGTH_ADDRESS 0x000004     // 新固件长度（4字节）
#define APP1_BOOT_COUNTER_ADDRESS 0x000008 // APP1启动计数器（4字节）
#define APP2_BOOT_COUNTER_ADDRESS 0x00000C // APP2启动计数器（4字节）
#define ACTIVE_APP_FLAG_ADDRESS                                                \
  0x000010 // 当前活动APP标志（4字节：1=APP1, 2=APP2）
#define ROLLBACK_FLAG_ADDRESS 0x000014 // 回滚标志（4字节）
#define APP1_VERSION_ADDRESS 0x000018  // APP1版本号（4字节）
#define APP2_VERSION_ADDRESS 0x00001C  // APP2版本号（4字节）

// ========== W25Q128 Flash分区地址 ==========
#define OTA_DOWNLOAD_START_ADDR                                                \
                               0x010000  // OTA下载缓冲区起始地址 (1024KB)
#define APP1_BACKUP_START_ADDR 0x100000 // APP1镜像备份区（1MB处）
#define APP2_BACKUP_START_ADDR 0x200000 // APP2镜像备份区（2MB处，可选）

// ========== STM32内部Flash地址（仅供参考，实际由链接脚本定义）==========
#define BOOTLOADER_START_ADDR 0x08000000 // Bootloader起始地址（32KB）
#define APP1_START_ADDR 0x08010000       // APP1区起始地址（448KB，sector4-7）
#define APP2_START_ADDR 0x08080000       // APP2区起始地址（512KB，sector8-11）

// ========== 魔术字和常量定义 ==========
#define OTA_FLAG_MAGIC_WORD 0xAA55AA55      // OTA升级标志魔术字
#define OTA_FLAG_NONE_WORD 0xFFFFFFFF       // 无升级标志
#define ROLLBACK_FLAG_MAGIC_WORD 0xBB66BB66 // 回滚标志魔术字
#define ACTIVE_APP_1 1                      // APP1活动
#define ACTIVE_APP_2 2                      // APP2活动

// ========== 启动失败阈值 ==========
#define MAX_BOOT_ATTEMPTS 3 // 最大启动尝试次数（3次失败后回滚）

// ========== 分区大小定义 ==========
#define APP1_MAX_SIZE (448 * 1024)         // APP1区最大448KB(Sector 4-7)
#define APP2_MAX_SIZE (512 * 1024)         // APP2区最大512KB(Sector 8-11)
#define OTA_DOWNLOAD_MAX_SIZE (1024 * 1024) // OTA下载区最大1MB

// ==================== 旧定义保留（向后兼容）====================
#define OTA_FIRMWARE_START_ADDR OTA_DOWNLOAD_START_ADDR // 兼容旧代码
#define BACKUP_APP_START_ADDR APP1_BACKUP_START_ADDR    // 兼容旧代码
#define BOOT_COUNTER_ADDRESS APP1_BOOT_COUNTER_ADDRESS  // 兼容旧代码
#define BOOT_COUNTER_SECTOR_ADDR 0x000000               // 启动计数器所在扇区

// ==================== OTA状态机定义 ====================

typedef enum {
  OTA_STATE_IDLE,        // 空闲状态
  OTA_STATE_CHECKING,    // 检查版本
  OTA_STATE_ERASING,     // 擦除Flash（分片执行）
  OTA_STATE_DOWNLOADING, // 下载固件数据
  OTA_STATE_COMPLETE     // 下载完成，等待重启
} OTA_State_t;

// ==================== 公开函数声明 ====================

/**
 * @brief  OTA模块初始化
 * @param  None
 * @retval None
 * @note   创建W25Q128互斥锁，必须在main()中FreeRTOS启动前调用
 */
void OTA_Init(void);

/**
 * @brief  确认启动成功（清零启动计数器）
 * @param  None
 * @retval None
 * @note   必须在main()中调用，告诉Bootloader"APP正常运行"
 */
void OTA_ConfirmStartup(void);

/**
 * @brief  处理MQTT OTA命令
 * @param  cmd: MQTT命令字符串
 * @param  len: 命令长度
 * @retval None
 * @note   在Command_Task中调用，处理upcheck、up in、UPFILE等命令
 */
void OTA_ProcessMQTTCommand(const char *cmd, uint16_t len);

/**
 * @brief  OTA任务初始化（创建OTA_Task）
 * @param  None
 * @retval None
 * @note   在app_init_task中调用
 */
void OTA_TaskInit(void);

/**
 * @brief  发送命令给DTU (通过UART4)
 * @param  cmd: 命令字符串（必须包含\r\n）
 * @retval None
 * @note   用于主动发送OTA命令，如"config,get,upcheck\r\n"
 */
void OTA_SendCommand(const char *cmd);

/**
 * @brief  检查是否正在进行OTA升级
 * @param  None
 * @retval true=正在OTA升级, false=空闲状态
 * @note   供其他任务查询OTA状态，避免UART4冲突
 */
bool OTA_IsInProgress(void);

/**
 * @brief  注册需要在OTA期间暂停的任务句柄
 * @param  acq_handle: 采集任务句柄
 * @param  upload_handle: 上传任务句柄
 * @param  app_handle: APP业务任务句柄
 * @param  oled_handle: OLED任务句柄
 * @param  heartbeat_handle: 心跳任务句柄
 * @param  watchdog_handle: 看门狗任务句柄
 * @retval None
 * @note   在app_init_task创建任务后调用，传入NULL表示该任务不存在
 */
void OTA_RegisterTasks(TaskHandle_t acq_handle, TaskHandle_t upload_handle,
                       TaskHandle_t app_handle, TaskHandle_t oled_handle,
                       TaskHandle_t heartbeat_handle,
                       TaskHandle_t watchdog_handle);

/**
 * @brief  OTA任务函数（状态机执行）
 * @param  pvParameters: 任务参数（未使用）
 * @retval None
 * @note   由OTA_TaskInit创建，不要直接调用
 */
void OTA_Task(void *pvParameters);

#endif /* __OTA_H */
