#include "HeartBeatTask.h"
#include "FreeRTOS.h"
#include "AcqTask.h"
#include "WQInterface.h"
#include "task.h"
#include "uart4.h"
#include <stdio.h>
#include <string.h>

/* 心跳JSON缓冲区 */
static char heartbeat_buffer[HEARTBEAT_BUFFER_SIZE];

static void Heartbeat_FormatJSON(char *buffer, uint16_t buffer_size);
static void Heartbeat_SendData(const char *data);

/**
 * @brief  初始化心跳模块底层（可选）
 */
void Heartbeat_Init(void) {
  // 目前无需特殊初始化，UART4已在系统底层统一激活
}

/**
 * @brief  心跳任务函数
 * @note   每 240 秒固定下发一次包含设备基本参数的轻量级 JSON 宣示保活在线
 */
void Heartbeat_Task(void *pvParameters) {
  TickType_t last_wake_time;

  last_wake_time = xTaskGetTickCount();

  while (1) {
    /* 1. 格式化心跳JSON */
    Heartbeat_FormatJSON(heartbeat_buffer, HEARTBEAT_BUFFER_SIZE);

    /* 2. 发送心跳数据 */
    Heartbeat_SendData(heartbeat_buffer);

    /* 3. 等待下一个心跳周期（固定240秒） */
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HEARTBEAT_INTERVAL * 1000));
  }
}

/**
 * @brief  格式化心跳JSON数据
 * @param  buffer: 输出缓冲区
 * @param  buffer_size: 缓冲区大小
 * @note   轻量级JSON，只包含关键状态信息
 */
static void Heartbeat_FormatJSON(char *buffer, uint16_t buffer_size) {
  char time_str[30];

  /* 引用采集模块的全局变量获取最新数据 */
  extern acquisition_data_t g_latest_acquisition_data;
  extern SemaphoreHandle_t g_acquisition_data_mutex;

  acquisition_data_t acq_data_local;

  /* 获取最新采集数据（需要互斥锁保护） */
  if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    memcpy(&acq_data_local, &g_latest_acquisition_data, sizeof(acquisition_data_t));
    xSemaphoreGive(g_acquisition_data_mutex);
  } else {
    acq_data_local.battery = 0;
    acq_data_local.acq_frequency = 0;
  }

  /* 格式化时间字符串 */
  sprintf(time_str, "%04d.%02d.%02d %02d:%02d", acq_data_local.year,
          acq_data_local.month, acq_data_local.day, acq_data_local.hour,
          acq_data_local.min);

  /* 构建轻量级心跳JSON */
  sprintf(buffer, "\r\n{\r\n\"MessageType\":\"Heartbeat\",\r\n");
  sprintf(buffer + strlen(buffer), "\"DeviceID\":\"Text_20260127\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Number\":\"HDY1003\",\r\n");
  sprintf(buffer + strlen(buffer), "\"SN\":\"0641100302\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Time\":\"%s\",\r\n", time_str);
  sprintf(buffer + strlen(buffer), "\"Status\":\"Online\",\r\n");
  sprintf(buffer + strlen(buffer), "\"Battery\":%d,\r\n", acq_data_local.battery);
  sprintf(buffer + strlen(buffer), "\"AcqFrequency\":%d,\r\n", acq_data_local.acq_frequency);
  sprintf(buffer + strlen(buffer), "\"UploadFrequency\":%d\r\n", acq_data_local.upload_frequency);
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
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    /* 发送一个字节 */
    USART_SendData(UART4, (uint8_t)*p);
    p++;
  }

  /* 等待发送完成 */
  while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}
