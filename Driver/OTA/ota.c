//////////////////////////////////////////////////////////////////////////////////
// OTA固件升级模块实现 - FreeRTOS版本 (Refactored)
//
// 本文件实现OTA（Over-The-Air）固件升级的核心逻辑，包括：
//   1. 状态机管理（IDLE, CHECKING, DOWNLOADING, COMPLETE）
//   2. MQTT指令解析（upcheck / upin / UPFILE数据包）
//   3. Base64数据解码与 W25Q128 Flash 扇区化写入
//   4. 资源封锁机制：OTA期间自动挂起上传、采集等高并发任务，防止总线冲突
//   5. 健壮性：新增 CHECKING 状态 10s 超时保护，防止非法服务器回复导致系统卡死
//
// 关键实现细节：
//   ▶ 任务暂停机制：
//     - 收到"config,upin,ok"后调用 OTA_SuspendOtherTasks()
//     - 暂停：采集、上传、业务、OLED、心跳、看门狗任务
//     - 目的：确保 UART4 与 SPI1 总线在升级期间的独占访问
//
//   ▶ 下载保护：
//     - 60秒无数据自动退出下载模式并恢复系统任务
//     - 下载完成后由 Bootloader 负责固件校验与 APP 区域覆盖安装
//
// 创建日期: 2026/01/28
// 重构日期: 2026/04/19 (新增状态机超时保护与架构对齐)
//////////////////////////////////////////////////////////////////////////////////

#include "ota.h"
#include "task.h"
#include "version.h" // ← 新增：引入版本管理
#include "w25q128.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==================== 内部全局变量 ====================

static OTA_State_t g_ota_state = OTA_STATE_IDLE;             // OTA状态机
static uint32_t g_firmware_size = 0;                         // 新固件大小
static uint32_t g_downloaded_bytes = 0;                      // 已下载字节数
static uint32_t g_w25q_write_addr = OTA_FIRMWARE_START_ADDR; // 当前写入地址
static uint8_t g_page_buffer[256];                           // SPI页缓冲
static uint16_t g_page_buffer_idx = 0;                       // 页缓冲索引

// FreeRTOS资源
static SemaphoreHandle_t g_w25q_mutex = NULL; // W25Q128互斥锁

// 擦除进度
static uint32_t g_erase_sector_count = 0; // 需要擦除的扇区数
static uint32_t g_erased_sectors = 0;     // 已擦除的扇区数

// 超时检测
static uint32_t g_last_data_time = 0;     // 最后一次接收数据的时间（tick）
static uint32_t g_checking_start_time = 0; // 进入 CHECKING 状态的时间（tick）

// 任务管理（OTA期间暂停其他任务）
static TaskHandle_t g_acquisition_task_handle = NULL;
static TaskHandle_t g_upload_task_handle = NULL;
static TaskHandle_t g_app_task_handle = NULL;
static TaskHandle_t g_oled_task_handle = NULL;
static TaskHandle_t g_heartbeat_task_handle = NULL; // ← 心跳包任务句柄
static TaskHandle_t g_watchdog_task_handle = NULL;  // ← 看门狗任务句柄

// ==================== 内部辅助函数 ====================

/**
 * @brief  解析版本字符串为整数（用于比较）
 * @param  version_str 版本字符串（如"V1.2.3"或"1.2.3"）
 * @return 版本号（格式：0xMMNNPP00，MM=Major, NN=Minor, PP=Patch）
 */
static uint32_t ParseVersionString(const char *version_str) {
  if (version_str == NULL) {
    return 0;
  }

  // 跳过'V'前缀
  const char *p = version_str;
  if (*p == 'V' || *p == 'v') {
    p++;
  }

  // 解析 major.minor.patch
  uint32_t major = 0, minor = 0, patch = 0;

  major = strtoul(p, (char **)&p, 10);
  if (*p == '.') {
    p++;
    minor = strtoul(p, (char **)&p, 10);
  }
  if (*p == '.') {
    p++;
    patch = strtoul(p, (char **)&p, 10);
  }

  return (major << 24) | (minor << 16) | (patch << 8);
}

/**
 * @brief  暂停其他任务（OTA期间）
 */
static void OTA_SuspendOtherTasks(void) {
  printf("[OTA] Suspending other tasks for OTA upgrade...\r\n");

  if (g_acquisition_task_handle != NULL) {
    vTaskSuspend(g_acquisition_task_handle);
    printf("[OTA]   - Acquisition task suspended\r\n");
  }

  if (g_upload_task_handle != NULL) {
    vTaskSuspend(g_upload_task_handle);
    printf("[OTA]   - Upload task suspended\r\n");
  }

  if (g_app_task_handle != NULL) {
    vTaskSuspend(g_app_task_handle);
    printf("[OTA]   - APP task suspended\r\n");
  }

  if (g_oled_task_handle != NULL) {
    vTaskSuspend(g_oled_task_handle);
    printf("[OTA]   - OLED task suspended\r\n");
  }

  if (g_heartbeat_task_handle != NULL) {
    vTaskSuspend(g_heartbeat_task_handle);
    printf("[OTA]   - Heartbeat task suspended\r\n");
  }

  if (g_watchdog_task_handle != NULL) {
    vTaskSuspend(g_watchdog_task_handle);
    printf("[OTA]   - Watchdog task suspended\r\n");
  }

  printf("[OTA] All non-critical tasks suspended.\r\n");
}

/**
 * @brief  恢复其他任务（OTA失败或取消时）
 */
static void OTA_ResumeOtherTasks(void) {
  printf("[OTA] Resuming other tasks...\r\n");

  if (g_acquisition_task_handle != NULL) {
    vTaskResume(g_acquisition_task_handle);
    printf("[OTA]   - Acquisition task resumed\r\n");
  }

  if (g_upload_task_handle != NULL) {
    vTaskResume(g_upload_task_handle);
    printf("[OTA]   - Upload task resumed\r\n");
  }

  if (g_app_task_handle != NULL) {
    vTaskResume(g_app_task_handle);
    printf("[OTA]   - APP task resumed\r\n");
  }

  if (g_oled_task_handle != NULL) {
    vTaskResume(g_oled_task_handle);
    printf("[OTA]   - OLED task resumed\r\n");
  }

  if (g_heartbeat_task_handle != NULL) {
    vTaskResume(g_heartbeat_task_handle);
    printf("[OTA]   - Heartbeat task resumed\r\n");
  }

  if (g_watchdog_task_handle != NULL) {
    vTaskResume(g_watchdog_task_handle);
    printf("[OTA]   - Watchdog task resumed\r\n");
  }

  printf("[OTA] All tasks resumed.\r\n");
}

// ==================== 内部辅助函数 ====================

/**
 * @brief  刷新页缓冲（写入W25Q128）
 */
static void Flush_Page_Buffer(void) {
  if (g_page_buffer_idx > 0) {
    xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
    W25Q128_PageProgram(g_w25q_write_addr, g_page_buffer, g_page_buffer_idx);
    xSemaphoreGive(g_w25q_mutex);

    g_w25q_write_addr += g_page_buffer_idx;
    g_page_buffer_idx = 0;
  }
}

/**
 * @brief  处理固件数据包
 * @param  data: 数据指针
 * @param  len: 数据长度
 */
static void ProcessFirmwareData(uint8_t *data, uint16_t len) {
  // ⚠️ 调试1：打印多个数据包的前16字节
  static uint32_t packet_count = 0;
  packet_count++;

  // 打印第1包、第5包、第10包
  if ((packet_count == 1 || packet_count == 5 || packet_count == 10) &&
      len >= 16) {
    printf("=== [Packet %lu] UART RX first 16 bytes ===\r\n", packet_count);
    for (int i = 0; i < 16; i++) {
      printf("%02X ", data[i]);
    }
    printf("\r\n");
    if (packet_count == 1) {
      printf(
          "Expected:     40 07 00 20 29 8F 00 08 15 84 00 08 11 84 00 08\r\n");
    }
    printf("===========================================\r\n");
  }

  for (uint16_t i = 0; i < len; i++) {
    g_page_buffer[g_page_buffer_idx++] = data[i];
    g_downloaded_bytes++;
    if (g_page_buffer_idx == 256) {
      uint32_t write_addr = g_w25q_write_addr;

      xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
      W25Q128_PageProgram(write_addr, g_page_buffer, 256);
      xSemaphoreGive(g_w25q_mutex);

      // ⚠️ 调试2：写入后立即回读验证（第1、5、10页）
      if (write_addr == OTA_FIRMWARE_START_ADDR ||
          write_addr == OTA_FIRMWARE_START_ADDR + 256 * 4 ||
          write_addr == OTA_FIRMWARE_START_ADDR + 256 * 9) {

        uint8_t verify[16];
        xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
        W25Q128_ReadData(write_addr, verify, 16);
        xSemaphoreGive(g_w25q_mutex);

        printf("=== W25Q128 Verify [0x%06lX] ===\r\n", write_addr);
        printf("Write: ");
        for (int i = 0; i < 16; i++)
          printf("%02X ", g_page_buffer[i]);
        printf("\r\nRead:  ");
        for (int i = 0; i < 16; i++)
          printf("%02X ", verify[i]);

        bool match = true;
        for (int i = 0; i < 16; i++) {
          if (g_page_buffer[i] != verify[i]) {
            match = false;
            break;
          }
        }
        printf("\r\nResult: %s\r\n", match ? "MATCH" : "MISMATCH");
        printf("=================================\r\n");
      }
      g_w25q_write_addr += 256;
      g_page_buffer_idx = 0;
    }
  }
  g_last_data_time = xTaskGetTickCount();
}

// ==================== 公开函数实现 ====================

/**
 * @brief  OTA模块初始化
 */
void OTA_Init(void) {
  // 创建W25Q128互斥锁
  g_w25q_mutex = xSemaphoreCreateMutex();
  if (g_w25q_mutex == NULL) {
    printf("[OTA] Failed to create W25Q128 mutex!\r\n");
  }
}

/**
 * @brief  注册需要在OTA期间暂停的任务句柄
 */
void OTA_RegisterTasks(TaskHandle_t acq_handle, TaskHandle_t upload_handle,
                       TaskHandle_t app_handle, TaskHandle_t oled_handle,
                       TaskHandle_t heartbeat_handle,
                       TaskHandle_t watchdog_handle) {
  g_acquisition_task_handle = acq_handle;
  g_upload_task_handle = upload_handle;
  g_app_task_handle = app_handle;
  g_oled_task_handle = oled_handle;
  g_heartbeat_task_handle = heartbeat_handle;
  g_watchdog_task_handle = watchdog_handle; // ← 保存看门狗句柄

  printf("[OTA] Task handles registered:\r\n");
  printf("[OTA]   - Acquisition: %s\r\n", acq_handle ? "YES" : "NO");
  printf("[OTA]   - Upload: %s\r\n", upload_handle ? "YES" : "NO");
  printf("[OTA]   - APP: %s\r\n", app_handle ? "YES" : "NO");
  printf("[OTA]   - OLED: %s\r\n", oled_handle ? "YES" : "NO");
  printf("[OTA]   - Heartbeat: %s\r\n", heartbeat_handle ? "YES" : "NO");
  printf("[OTA]   - Watchdog: %s\r\n",
         watchdog_handle ? "YES" : "NO"); // ← 显示看门狗状态
}

/**
 * @brief  检查是否正在进行OTA升级
 */
bool OTA_IsInProgress(void) { return (g_ota_state != OTA_STATE_IDLE); }

/**
 * @brief  确认启动成功（清零启动计数器）
 */
void OTA_ConfirmStartup(void) {
  uint32_t counter;

  // 读取启动计数器
  xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
  W25Q128_ReadData(BOOT_COUNTER_ADDRESS, (uint8_t *)&counter, 4);
  xSemaphoreGive(g_w25q_mutex);

  // 如果计数器不为0，说明Bootloader记录了启动
  if (counter != 0 && counter != 0xFFFFFFFF) {
    printf("[OTA] Confirming startup, clearing boot counter...\r\n");

    // 擦除扇区并清零计数器
    xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
    W25Q128_SectorErase_4K(BOOT_COUNTER_SECTOR_ADDR);
    xSemaphoreGive(g_w25q_mutex);

    printf("[OTA] Startup confirmed.\r\n");
  }
}

/**
 * @brief  发送命令给DTU (通过UART4)
 * @param  cmd: 命令字符串（必须包含\r\n）
 * @note   参考裸机版本OTA.c第165行：Usart_SendString
 */
void OTA_SendCommand(const char *cmd) {
  if (cmd == NULL) {
    return;
  }

  // 通过UART4发送命令
  const char *p = cmd;
  while (*p) {
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
      ;
    USART_SendData(UART4, (uint8_t)*p);
    p++;
  }

  // 等待发送完成
  while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
    ;
}

/**
 * @brief  处理MQTT OTA命令
 */
void OTA_ProcessMQTTCommand(const char *cmd, uint16_t len) {
  // 1. 检查版本 - config,upcheck,ok,V1.1.1（参考裸机OTA.c第159-167行）
  if (strstr(cmd, "config,upcheck,ok") != NULL) {
    // 提取服务器版本号
    const char *version_start = strrchr(cmd, ',');
    if (version_start != NULL) {
      version_start++;
      uint32_t server_version = ParseVersionString(version_start);
      uint32_t current_version = Version_GetCode();

      printf("[OTA] Server: %s (0x%08lX)\r\n", version_start, server_version);
      printf("[OTA] Current: %s (0x%08lX)\r\n", Version_GetString(),
             current_version);

      if (server_version > current_version) {
        printf("[OTA] New version! Starting upgrade...\r\n");
        OTA_SendCommand("config,set,upin\r\n");
        g_ota_state = OTA_STATE_CHECKING;
      } else {
        printf("[OTA] Up-to-date, skip.\r\n");
        g_ota_state = OTA_STATE_IDLE;
      }
    }
  }
  // 2. 开始下载 - config,upin,ok
  else if (strstr(cmd, "config,upin,ok") != NULL) {
    printf("[OTA] *** Received upin OK, preparing to erase Flash ***\r\n");

    // ⚠️ 新增：暂停其他任务，防止资源冲突
    OTA_SuspendOtherTasks();

    // 重置下载状态
    g_w25q_write_addr = OTA_FIRMWARE_START_ADDR;
    g_downloaded_bytes = 0;
    g_page_buffer_idx = 0;

    g_firmware_size = 0; // 重置，等待第一个UPFILE包确定大小
    // ⚠️ 修复：直接进入DOWNLOADING状态，擦除将在收到第一个UPFILE包时执行
    g_ota_state = OTA_STATE_DOWNLOADING;

    printf("[OTA] Ready to receive UPFILE data packets...\r\n");
  }
  // 3. 接收固件数据 - UPFILE:<总长度>:<偏移量>:<包长度>:<数据>
  else if (strncmp(cmd, "UPFILE:", 7) == 0) {
    if (g_ota_state == OTA_STATE_DOWNLOADING) {
      // === 重要修复：准确定位二进制数据起始位置 ===
      char *ptr = (char *)cmd + 7; // 跳过"UPFILE:"

      // 解析总长度
      uint32_t total_len = strtoul(ptr, &ptr, 10);
      if (g_firmware_size == 0) {
        g_firmware_size = total_len; // 第一包时记录总大小
        printf("[OTA] Firmware size: %lu bytes\r\n", g_firmware_size);

        // ⚠️ 关键修复：在这里执行W25Q128擦除！
        g_erase_sector_count = (g_firmware_size + 4095) / 4096;
        printf("[OTA] Need to erase %lu sectors (4KB each)\r\n",
               g_erase_sector_count);

        // 立即擦除所有需要的扇区
        for (uint32_t i = 0; i < g_erase_sector_count; i++) {
          uint32_t sector_addr = OTA_FIRMWARE_START_ADDR + (i * 4096);
          xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
          W25Q128_SectorErase_4K(sector_addr);
          xSemaphoreGive(g_w25q_mutex);
          printf("[OTA] Erased sector %lu/%lu at 0x%06lX\r\n", i + 1,
                 g_erase_sector_count, sector_addr);
        }
        printf("[OTA] *** W25Q128 erase COMPLETE! ***\r\n");
      }

      ptr++; // 跳过冒号
      uint32_t current_offset =
          strtoul(ptr, &ptr, 10); // 偏移量（未使用，仅解析）

      ptr++;                                        // 跳过冒号
      uint16_t packet_len = strtoul(ptr, &ptr, 10); // 本包长度

      ptr++; // 跳过冒号，指向实际数据

      // 调试：输出包信息和处理前的字节数（注释掉以减少日志输出）
      // uint32_t bytes_before = g_downloaded_bytes;
      // printf("[OTA] UPFILE: offset=%lu, len=%u, before=%lu\r\n",
      // current_offset,
      //        packet_len, bytes_before);

      // 处理纯固件数据（完全按照裸机版本逻辑）
      ProcessFirmwareData((uint8_t *)ptr, packet_len);

      // 调试：输出处理后的字节数（注释掉以减少日志输出）
      // printf("[OTA] After process: %lu (delta=%lu)\r\n", g_downloaded_bytes,
      //        g_downloaded_bytes - bytes_before);

      // 打印进度（每1KB打印一次）
      if (g_downloaded_bytes % 1024 == 0) {
        printf("[OTA] Downloaded: %lu / %lu bytes (%.1f%%)\r\n",
               g_downloaded_bytes, g_firmware_size,
               (float)g_downloaded_bytes * 100.0f / g_firmware_size);
      }
    }
  }
  // 4. 下载完成 - config,upsta,ok,0（参考裸机OTA.c第237行）
  else if (strstr(cmd, "config,upsta,ok") != NULL) {
    // 提取状态码
    const char *status_str = strrchr(cmd, ',');
    int status_code = -1;
    if (status_str != NULL) {
      status_code = atoi(status_str + 1);
    }

    printf("[OTA] Received upsta, status code: %d\r\n", status_code);

    // 检查状态码（0=成功）
    if (status_code == 0) {
      // ===== 验证下载完整性 =====
      printf("[OTA] Verifying download completeness...\r\n");
      printf("[OTA] Downloaded: %lu bytes, Expected: %lu bytes\r\n",
             g_downloaded_bytes, g_firmware_size);

      // 防止0==0误判：固件大小必须大于0
      if (g_firmware_size == 0) {
        printf("[OTA] ERROR: Invalid firmware size (0)!\r\n");
        g_ota_state = OTA_STATE_IDLE;
      } else if (g_downloaded_bytes != g_firmware_size) {
        printf("[OTA] ERROR: Download INCOMPLETE! Size mismatch!\r\n");
        printf("[OTA] Only received %.1f%% of firmware data.\r\n",
               (float)g_downloaded_bytes * 100.0f / g_firmware_size);

        // 重置状态，不执行OTA
        g_ota_state = OTA_STATE_IDLE;
        g_firmware_size = 0;
        g_downloaded_bytes = 0;
      } else {
        printf("[OTA] Download SUCCESS! Size verified.\r\n");

        // 刷新剩余缓冲
        Flush_Page_Buffer();

        // 转换到COMPLETE状态
        g_ota_state = OTA_STATE_COMPLETE;
      }
    } else {
      printf("[OTA] Download FAILED! Status code: %d\r\n", status_code);
      printf("[OTA] Status codes: 1=no file, 2=crc error, 3=timeout/other\r\n");
      printf("[OTA] Downloaded: %lu bytes, Expected: %lu bytes\r\n",
             g_downloaded_bytes, g_firmware_size);

      // 重置状态
      g_ota_state = OTA_STATE_IDLE;
      g_firmware_size = 0;
      g_downloaded_bytes = 0;
    }
  }
}

/**
 * @brief  OTA任务函数（状态机执行）
 */
void OTA_Task(void *pvParameters) {
  uint32_t sector_addr;
  static uint32_t last_check_time = 0; // 上次检查OTA的时间

  while (1) {
    switch (g_ota_state) {
    case OTA_STATE_IDLE:
      // 空闲状态，每30秒自动检查OTA升级（参考裸机OTA.c第279-290行）
      {
        uint32_t current_time = xTaskGetTickCount();
        if (current_time - last_check_time > pdMS_TO_TICKS(30000)) { // 30秒
          last_check_time = current_time;

          printf("[OTA] Auto requesting firmware version check...\r\n");
          OTA_SendCommand("config,get,upcheck\r\n");
          g_ota_state = OTA_STATE_CHECKING;
          g_checking_start_time = xTaskGetTickCount(); // 记录进入 CHECKING 的时刻
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;

    case OTA_STATE_CHECKING:
      // 版本查询已发出，等待服务器回复 "config,upcheck,ok"
      // ★ 关键修复：加 10 秒超时保护，防止服务器回复 error 或无回复时
      //   g_ota_state 永远停在 CHECKING，导致 UploadTask/TFTask 被永久封啓。
      if ((xTaskGetTickCount() - g_checking_start_time) > pdMS_TO_TICKS(10000)) {
          printf("[OTA] Version check timeout (10s), server no valid response. "
                 "Back to IDLE.\r\n");
          g_ota_state = OTA_STATE_IDLE;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      break;

    case OTA_STATE_ERASING:
      // 分片擦除Flash（避免长时间阻塞）
      if (g_erased_sectors < g_erase_sector_count) {
        sector_addr = OTA_FIRMWARE_START_ADDR + (g_erased_sectors * 4096);

        xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
        W25Q128_SectorErase_4K(sector_addr);
        xSemaphoreGive(g_w25q_mutex);

        g_erased_sectors++;

        // 每擦除一个扇区都打印（帮助调试）
        printf("[OTA] Erasing sector %lu / %lu\r\n", g_erased_sectors,
               g_erase_sector_count);

        vTaskDelay(pdMS_TO_TICKS(50)); // 让出CPU
      } else {
        printf("[OTA] *** Flash erase COMPLETE! Waiting for firmware data... "
               "***\r\n");
        g_ota_state = OTA_STATE_DOWNLOADING;
        g_last_data_time = xTaskGetTickCount();
      }
      break;

    case OTA_STATE_DOWNLOADING:
      // 下载中，检查超时（60秒）
      if ((xTaskGetTickCount() - g_last_data_time) > pdMS_TO_TICKS(60000)) {
        printf("[OTA] Download timeout! Aborting...\r\n");

        // ⚠️ 新增：恢复其他任务
        OTA_ResumeOtherTasks();

        g_ota_state = OTA_STATE_IDLE;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      break;

    case OTA_STATE_COMPLETE:
      {
        // 下载完成，设置OTA标志并重启
        printf("[OTA] Setting OTA flag and rebooting...\r\n");

        // 写入OTA标志
        // ⚠️ 必须先擦除Sector 0！防止原来的数据干扰
        xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
        W25Q128_SectorErase_4K(0x000000); // 擦除第一个4KB扇区

        uint32_t ota_flag = OTA_FLAG_MAGIC_WORD;
        W25Q128_PageProgram(OTA_FLAG_ADDRESS, (uint8_t *)&ota_flag, 4);

        // ⚠️ 立即回读验证
        uint32_t check_flag = 0;
        W25Q128_ReadData(OTA_FLAG_ADDRESS, (uint8_t *)&check_flag, 4);
        if (check_flag != OTA_FLAG_MAGIC_WORD) {
          printf("[OTA] ERROR: Failed to write OTA flag! Read: 0x%08X, Expected: "
                 "0x%08X\r\n",
                 check_flag, OTA_FLAG_MAGIC_WORD);
          // 再次尝试擦除写入
          W25Q128_SectorErase_4K(0x000000);
          W25Q128_PageProgram(OTA_FLAG_ADDRESS, (uint8_t *)&ota_flag, 4);
        } else {
          printf("[OTA] OTA flag verified OK: 0x%08X\r\n", check_flag);
        }

        xSemaphoreGive(g_w25q_mutex);

        // 写入固件长度
        printf("[OTA] Writing firmware length: %lu bytes\r\n", g_firmware_size);
        xSemaphoreTake(g_w25q_mutex, portMAX_DELAY);
        W25Q128_PageProgram(OTA_FW_LENGTH_ADDRESS, (uint8_t *)&g_firmware_size, 4);
        xSemaphoreGive(g_w25q_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000)); // 等待写入完成

        printf("[OTA] Resetting system...\r\n");
        vTaskDelay(pdMS_TO_TICKS(500));

        // 系统复位
        NVIC_SystemReset();
      }
      break;


    default:
      g_ota_state = OTA_STATE_IDLE;
      break;
    }
  }
}

/**
 * @brief  OTA任务初始化（创建OTA_Task）
 */
void OTA_TaskInit(void) {
  xTaskCreate((TaskFunction_t)OTA_Task, (const char *)"ota_task",
              (uint16_t)512, // 512字堆栈
              (void *)NULL,
              (UBaseType_t)5, // 优先级5（中等）
              (TaskHandle_t *)NULL);

  printf("[OTA] OTA task created.\r\n");
}
