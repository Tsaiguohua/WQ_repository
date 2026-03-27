#include "heartbeat.h"
#include "FreeRTOS.h"
#include "AcqTask.h"
#include "gps.h"
#include "task.h"
#include "uart4.h"
#include <stdio.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// 设备心跳模块 - FreeRTOS版本
//
// 功能说明：
//   定期向MQTT服务器发送设备在线心跳消息，证明设备正常运行
//
// 工作流程：
//   1. 任务每240秒循环一次
//   2. 读取系统状态（时间、电量、采集频率等）
//   3. 构建轻量级JSON心跳消息
//   4. 通过UART4发送到4G模块
//
// JSON格式示例：
//   {
//     "MessageType": "Heartbeat",
//     "DeviceID": "WaterMonitor_001",
//     "Time": "2026.01.27 20:04",
//     "Status": "Online",
//     "Battery": 95,
//     "Frequency": 60
//   }
//
// 注意事项：
//   ⚠️ UART4被Upload_Task和Heartbeat_Task共享
//   ⚠️ 输出使用UART4硬件直接发送，与printf互不干扰
//
// 创建日期：2026/01/27
// 作者：蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          私有变量
 *===========================================================================*/

/* 心跳JSON缓冲区 */
static char heartbeat_buffer[HEARTBEAT_BUFFER_SIZE];

/*============================================================================
 *                          私有函数声明
 *===========================================================================*/

static void Heartbeat_FormatJSON(char *buffer, uint16_t buffer_size);
static void Heartbeat_SendData(const char *data);

/*============================================================================
 *                          API实现
 *===========================================================================*/

/**
 * @brief  初始化心跳模块
 */
void Heartbeat_Init(void) {
  // 目前无需特殊初始化
  // UART4已在main中初始化
}

/**
 * @brief  初始化心跳任务
 * @retval 心跳任务句柄
 */
TaskHandle_t Heartbeat_TaskInit(void) {
  /* 创建心跳任务 */
  TaskHandle_t heartbeat_task_handle = NULL;      // ← 保存句柄
  BaseType_t result = xTaskCreate(Heartbeat_Task, // 任务函数
                                  "heartbeat",    // 任务名称
                                  512,  // 堆栈512字(2048B)，心跳消息较小
                                  NULL, // 任务参数
                                  2,    // 优先级（与Upload_Task同级或更低）
                                  &heartbeat_task_handle // ← 传出句柄
  );

  if (result != pdPASS) {
    while (1)
      ; // 任务创建失败
  }

  return heartbeat_task_handle; // ← 返回句柄
}

/**
 * @brief  心跳任务函数
 * @note   每240秒发送一次心跳消息
 */
void Heartbeat_Task(void *pvParameters) {
  TickType_t last_wake_time;

  /* 初始化时间基准 */
  last_wake_time = xTaskGetTickCount();

  // printf("[Heartbeat] Task started, interval=%d sec\r\n",
  // HEARTBEAT_INTERVAL);

  while (1) {
    /* 1. 格式化心跳JSON */
    Heartbeat_FormatJSON(heartbeat_buffer, HEARTBEAT_BUFFER_SIZE);

    /* 2. 发送心跳数据 */
    Heartbeat_SendData(heartbeat_buffer);

    // printf("[Heartbeat] Sent (next in %ds)\r\n", HEARTBEAT_INTERVAL);

    /* 3. 等待下一个心跳周期（固定240秒） */
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HEARTBEAT_INTERVAL * 1000));
  }
}

/*============================================================================
 *                          私有函数实现
 *===========================================================================*/

/**
 * @brief  格式化心跳JSON数据
 * @param  buffer: 输出缓冲区
 * @param  buffer_size: 缓冲区大小
 * @note   轻量级JSON，只包含关键状态信息
 */
static void Heartbeat_FormatJSON(char *buffer, uint16_t buffer_size) {
  //  gps_data_t gps_data;
  //  bool gps_valid = false;
  char time_str[30];

  /* 引用采集模块的全局变量获取最新数据 */
  extern acquisition_data_t g_latest_acquisition_data;
  extern SemaphoreHandle_t g_acquisition_data_mutex;

  acquisition_data_t acq_data_local;

  /* 获取最新采集数据（需要互斥锁保护） */
  if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    memcpy(&acq_data_local, &g_latest_acquisition_data,
           sizeof(acquisition_data_t));
    xSemaphoreGive(g_acquisition_data_mutex);
  } else {
    /* 如果获取失败，使用默认值 */
    // printf("[Heartbeat] Failed to acquire mutex, using default values\r\n");
    acq_data_local.battery = 0;
    acq_data_local.acq_frequency = 0;
  }

  /* 获取GPS数据 */
  //  gps_valid = GPS_GetData(&gps_data);

  /* 格式化时间字符串 */
  sprintf(time_str, "%04d.%02d.%02d %02d:%02d", acq_data_local.year,
          acq_data_local.month, acq_data_local.day, acq_data_local.hour,
          acq_data_local.min);

  /* 构建轻量级心跳JSON（分步构建） */
  sprintf(buffer, "\r\n{\r\n\"MessageType\":\"Heartbeat\",\r\n");
  sprintf(buffer + strlen(buffer), "\"DeviceID\":\"Text_20260127\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Number\":\"HDY1003\",\r\n");
  sprintf(buffer + strlen(buffer), "\"SN\":\"0641100302\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Time\":\"%s\",\r\n", time_str);
  sprintf(buffer + strlen(buffer), "\"Status\":\"Online\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Battery\":%d,\r\n",
          acq_data_local.battery);
  // sprintf(buffer + strlen(buffer), "\"GPS\":%d,\r\n", gps_valid ? 1 : 0);
  sprintf(buffer + strlen(buffer), "\"AcqFrequency\":%d,\r\n",
          acq_data_local.acq_frequency);
  sprintf(buffer + strlen(buffer), "\"UploadFrequency\":%d\r\n",
          acq_data_local.upload_frequency);
  sprintf(buffer + strlen(buffer), "}\r\n");
}

/**
 * @brief  发送心跳数据到4G模块（通过UART4）
 * @note   直接操作UART4硬件发送，避免与printf调试输出冲突
 */
static void Heartbeat_SendData(const char *data) {
  if (data == NULL) {
    return;
  }

  /* 通过UART4发送数据 */
  const char *p = data;
  while (*p) {
    /* 等待发送缓冲区空 */
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
      ;
    /* 发送一个字节 */
    USART_SendData(UART4, (uint8_t)*p);
    p++;
  }

  /* 等待发送完成 */
  while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
    ;
}
