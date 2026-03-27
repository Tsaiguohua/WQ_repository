#include "app.h"
#include "FreeRTOS.h"
#include "acquisition.h" // 采集任务
#include "aht20.h"       // AHT20驱动
#include "aht20_desc.h"  // AHT20驱动
// #include "button.h"   // 面包板测试时暂时注释：按键驱动

#include "gps.h"       // GPS驱动
#include "heartbeat.h" // 心跳任务
#include "oled.h"      // OLED驱动
#include "semphr.h"    // 信号量支持
#include "task.h"
#include "uart4.h"    // UART4双缓冲支持
#include "upload.h"   // 上传任务
#include "voltage.h"  // 电量检测
#include "watchdog.h" // 看门狗驱动
#include <stdbool.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// APP应用层任务管理模块 - FreeRTOS版本
//
// ⚠️ 重构状态说明（2026/03/18）：
//   本文件的任务管理职责已迁移到新架构文件，本文件处于《迁移期/可废弃》状态：
//
//   【已迁移——本文件中对应函数不再被调用】
//   - app_init_task（创建所有任务）→ User/Tasks/TasksInit.c::User_Tasks_Init()
//   - app_oled_task（OLED刷新任务）→ User/Tasks/TasksInit.c::OledTask()
//
//   【可以删除——AcqTask.c 中已有等效实现】
//   - app_task（AHT20+电量采集）→ AcqTask.c::collect_environment_data()
//
// 下一步行动：
//   确认新架构正常运行后，可以整个删除 app.c 和 app.h。
//
// 作者：蔡国华
// 创建日期：2025/12/30
// 最后更新：2026/03/18（标注已废弃，职责已迁移到 TasksInit.c）
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          任务参数定义
 *===========================================================================*/

/* OLED控制任务 */
#define OLED_CTRL_TASK_PRIO 4
#define OLED_CTRL_STK_SIZE 640 // 640字(2560B)，OLED刷新+printf

/* 采集任务 */
#define ACQ_TASK_PRIO 6
#define ACQ_STK_SIZE 640 // 640字(2560B)，传感器采集+sprintf

/* APP业务任务 */
#define APP_TASK_PRIO 5
#define APP_STK_SIZE 384 // 384字(1536B)，AHT20+电量+printf

/* Command指令处理任务 */
#define CMD_TASK_PRIO 7
#define CMD_STK_SIZE 640 // 640字(2560B)，MQTT指令解析+printf

/*============================================================================
 *                          业务任务实现
 *===========================================================================*/

/**
 * @brief  APP业务任务
 * @note   处理环境数据的定期采集（AHT20温湿度 + 电量）
 */
void app_task(void *pvParameters) {
  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t bat_percentage = 0;

  // 初始化AHT20
  if (!aht20_init(AHT20_DEFAULT)) {
    // printf("[APP] AHT20 Init failed!\r\n");
  } else {
    // printf("[APP] AHT20 Init success\r\n");
  }

  // 初始化电量检测
  Voltage_Init();
  // printf("[APP] Voltage Init success\r\n");

  // 任务主循环
  while (1) {
    /* 1. 读取AHT20温湿度数据 */
    if (aht20_read_temp_humi(AHT20_DEFAULT, &temperature, &humidity)) {
      // printf("[APP] Temp: %.1f C, Humi: %.1f %%\r\n", temperature, humidity);
    } else {
      // printf("[APP] AHT20 read failed!\r\n");
    }

    /* 2. 读取电量数据 */
    bat_percentage = Voltage_GetBattery();
    // printf("[APP] Battery: %d %%\r\n", bat_percentage);

    /* 3. 检查低电量 */
    if (Voltage_IsLow()) {
      // printf("[APP] Warning: Low battery!\r\n");
    }

    /* 打印分隔线 */
    // printf("----------------------------------------\r\n");

    /* TODO: 后续可以将数据通过队列发送给上传任务
     * 或者存储到全局变量供上传任务读取
     *
     * 示例：
     * env_data_t env_data = {
     *   .temperature = temperature,
     *   .humidity = humidity,
     *   .battery = bat_percentage
     * };
     * xQueueSend(upload_queue, &env_data, 0);
     */

    // 4秒采集一次
    vTaskDelay(pdMS_TO_TICKS(4000));
  }
}

/*============================================================================
 *                          任务创建和管理
 *===========================================================================*/

/**
 * @brief  APP初始化任务（启动任务）
 * @note   负责创建所有业务任务，完成后自删除
 */
void app_init_task(void *pvParameters) {
  printf("\r\n========== APP Started from Bootloader ==========\r\n");

  // ⚠️ OLED_Init()必须在任务中调用，因为U8g2库消耗超过4096字节堆栈
  extern void OLED_Init(void);
  OLED_Init();
  // printf("[APP] OLED initialized in task\r\n");

  // 初始化RS485通信 (创建互斥锁和信号量)
  extern bool rs485_init(void);
  if (rs485_init()) {
    // printf("[APP] RS485 initialized\r\n");
  } else {
    // printf("[APP] RS485 init failed!\r\n");
  }

  // 初始化自检模块（配置通道控制GPIO）
  //  extern void SelfExam_Init(void);
  //  SelfExam_Init();
  // printf("[APP] SelfExam module initialized\r\n");

  // 执行传感器自检（必须在FreeRTOS启动后，因为内部使用vTaskDelay）
  //  extern void SelfExam_StartExaminate(void);
  //  SelfExam_StartExaminate();
  // printf("[APP] SelfExam completed\r\n");

  taskENTER_CRITICAL(); // 进入临界区

  // 创建GPS解析任务
  //  GPS_TaskInit();

  // 创建上传任务（定期读取全局数据并上传）
  //  Upload_TaskInit();

  // 创建心跳任务（240秒周期，独立于数据上传）
  //  Heartbeat_TaskInit();

  // 创建OTA任务（优先级5，处理固件升级）
  extern void OTA_TaskInit(void);
  OTA_TaskInit();

  // 创建看门狗任务（最低优先级，监控系统运行）
  //  Watchdog_TaskInit();

  // OTA任务句柄（用于OTA期间暂停/恢复）
  static TaskHandle_t acq_handle = NULL;
  static TaskHandle_t upload_handle = NULL;
  static TaskHandle_t app_handle = NULL;
  static TaskHandle_t oled_handle = NULL;
  static TaskHandle_t heartbeat_handle = NULL;
  static TaskHandle_t watchdog_handle = NULL; // ← 看门狗句柄

  // ========== OTA双APP测试模式：禁用所有非OTA任务 ==========
  printf("[APP] *** OTA TEST MODE: Non-OTA tasks disabled ***\r\n");

  /*
  // 创建APP业务任务（处理环境数据比如AHT20、电量等）
  xTaskCreate((TaskFunction_t)app_task, (const char *)"app_task",
              (uint16_t)APP_STK_SIZE, (void *)NULL, (UBaseType_t)APP_TASK_PRIO,
              &app_handle);

  // 创建OLED控制任务
  xTaskCreate((TaskFunction_t)app_oled_task, (const char *)"oled_ctrl",
              (uint16_t)OLED_CTRL_STK_SIZE, (void *)NULL,
              (UBaseType_t)OLED_CTRL_TASK_PRIO, &oled_handle);

  // 创建水质传感器采集任务（更新全局共享数据）
  xTaskCreate((TaskFunction_t)Acquisition_Task, (const char *)"acq_task",
              (uint16_t)ACQ_STK_SIZE, (void *)NULL, (UBaseType_t)ACQ_TASK_PRIO,
              &acq_handle);

  // 创建数据上传任务
  extern TaskHandle_t Upload_TaskInit(void);
  upload_handle = Upload_TaskInit();

  // 创建心跳任务（240秒周期，独立于数据上传）
  extern TaskHandle_t Heartbeat_TaskInit(void);
  heartbeat_handle = Heartbeat_TaskInit();

  // 创建看门狗任务（最低优先级，15秒喂狗）
  extern TaskHandle_t Watchdog_TaskInit(void);
  watchdog_handle = Watchdog_TaskInit();
  */

  // 创建 UART接收队列（用于双缓冲机制，替代原信号量）
  extern QueueHandle_t xUartRxQueue;
  xUartRxQueue = xQueueCreate(5, sizeof(UartRxMsg_t));
  if (xUartRxQueue == NULL) {
    // printf("[APP] UART RX queue create failed!\r\n");
  }

  // 创建MQTT指令处理任务
  xTaskCreate((TaskFunction_t)Command_Task, (const char *)"cmd_task",
              (uint16_t)CMD_STK_SIZE, (void *)NULL, (UBaseType_t)CMD_TASK_PRIO,
              (TaskHandle_t *)NULL);

  // ⚠️ 注册任务句柄到OTA模块（测试模式：全部NULL）
  extern void OTA_RegisterTasks(TaskHandle_t acq, TaskHandle_t upload,
                                TaskHandle_t app, TaskHandle_t oled,
                                TaskHandle_t heartbeat, TaskHandle_t watchdog);
  OTA_RegisterTasks(NULL, NULL, NULL, NULL, NULL, NULL); // ← 全部NULL

  printf("All tasks created. APP running normally.\r\n");
  printf("=================================================\r\n\r\n");

  taskEXIT_CRITICAL(); // 退出临界区

  vTaskDelete(NULL); // 删除自己（启动任务完成）
}

/**
 * @brief  OLED控制任务
 * @note   面包板测试版本：直接显示OLED，不需要按键控制
 *         - OLED上电后直接显示开机动画和主界面
 *         - 固定1秒刷新周期，及时显示温湿度和上传频率的变化
 */
void app_oled_task(void *pvParameters) {
  static bool boot_animation_shown =
      false; // 开机动画是否已显示（static保持状态）

  while (1) {
    /* 面包板测试：直接显示OLED，不检测按键 */

    /* 第一次显示：开机动画 */
    if (!boot_animation_shown) {
      OLED_ShowBootAnimation(); // 进度条0-100%
      boot_animation_shown = true;
    }

    /* 显示主界面（时间、温湿度、电量等） */
    OLED_Update();

    /* 固定1秒刷新周期：及时显示数据变化，包括上传频率的更新 */
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* 原始按键控制版本（测试通过后恢复）
void app_oled_task(void *pvParameters) {
  static bool boot_animation_shown =
      false; // 开机动画是否已显示（static保持状态）

  while (1) {
    // 检测按键电平状态
    if (Button_GetState() == 0) {
      // 按键按下（低电平）- OLED通电

      // 第一次OLED通电：显示开机动画
      if (!boot_animation_shown) {
        OLED_ShowBootAnimation(); // 进度条0-100%
        boot_animation_shown = true;
      }

      // 显示主界面（时间、温湿度、电量等）
      OLED_Update();

    } else {
      // 按键释放（高电平）- OLED断电，熄灭屏幕
      OLED_Clear();
    }

    // 50ms扫描周期
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
*/
