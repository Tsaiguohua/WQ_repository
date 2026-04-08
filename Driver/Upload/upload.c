#include "upload.h"
#include "FreeRTOS.h"
#include "AcqTask.h"
#include "gps.h"
#include "self_exam.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// MQTT数据上传任务模块 - FreeRTOS版本
//
// 功能说明：
//   本模块负责周期性将采集的数据打包成JSON格式，通过4G模块上传到MQTT服务器
//
// 硬件接口：
//   - 通信接口：UART4 (PC10=TX, PC11=RX)
//   - 波特率：115200
//   - 4G模块：透传模式，MQTT已配置
//
// JSON数据格式：
//   包含设备ID、环境数据（温湿度、电量）、GPS数据、水质传感器数据
//   示例：{"ID":3,"Number":"HDY1003","Temp":"25.6","Humi":"60.2",...}
//
// 数据来源：
//   从采集任务的全局变量g_latest_acquisition_data读取
//   读取时使用互斥锁保护，确保数据一致性
//
// 频率控制：
//   - 默认上传频率：10秒
//   - 可通过MQTT命令动态修改：{"UploadFrequency": 30}
//   - 修改后自动保存到Flash，重启后恢复
//   - 范围：10~3600秒
//
// 传感器状态处理：
//   - 已连接传感器：上传实际测量值
//   - 未连接传感器：上传999（区分"数据为0"和"未连接"）
//
// 使用流程：
//   1. Upload_TaskInit()创建上传任务（app_init_task中调用）
//   2. 任务按上传频率周期性运行
//   3. 读取采集数据 → 格式化JSON → 通过UART4发送
//
// 注意事项：
//   ⚠️ UART4专用于4G模块，不要用于其他用途
//   ⚠️ 读取采集数据时必须通过互斥锁保护
//   ⚠️ JSON字符串缓冲区1200字节，注意不要溢出
//
// 作者：蔡国华
// 创建日期：2026/01/04
// 最后更新：2026/01/11
// 版本：v1.0
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          全局变量
 *===========================================================================*/

/* 当前上传频率（秒） */
uint16_t g_upload_frequency = UPLOAD_FREQ_DEFAULT;

/* 上传缓冲区 */
static char upload_buffer[UPLOAD_BUFFER_SIZE];

/* 立即上传标志 */
static volatile bool upload_now_flag = false;

/* 互斥量（保护上传频率） */
static SemaphoreHandle_t upload_freq_mutex = NULL;

/*============================================================================
 *                          私有函数声明
 *===========================================================================*/

static void Upload_FormatJSON(char *buffer, uint16_t buffer_size,
                              acquisition_data_t *acq_data);
static void Upload_SendData(const char *data);

/*============================================================================
 *                          API实现
 *===========================================================================*/

/**
 * @brief  初始化上传模块
 */
void Upload_Init(void) {
  // 这里可以初始化4G模块（如果需要）
  // 目前假设4G模块已经在透传模式下配置好MQTT
}

/**
 * @brief  初始化上传任务
 * @retval 上传任务句柄
 */
TaskHandle_t Upload_TaskInit(void) {
  /* 创建互斥量 */
  upload_freq_mutex = xSemaphoreCreateMutex();
  if (upload_freq_mutex == NULL) {
    while (1)
      ; // 互斥量创建失败
  }

  /* 创建上传任务 */
  TaskHandle_t upload_task_handle = NULL;      // ← 保存句柄
  BaseType_t result = xTaskCreate(Upload_Task, // 任务函数
                                  "upload",    // 任务名称
                                  768,  // 堆栈768字(3072B)，JSON格式化需要
                                  NULL, // 任务参数
                                  3,    // 优先级（低于采集任务）
                                  &upload_task_handle // ← 传出句柄
  );

  if (result != pdPASS) {
    while (1)
      ; // 任务创建失败
  }

  return upload_task_handle; // ← 返回句柄
}

/**
 * @brief  上传任务函数
 * @note   按上传频率定期从全局变量读取最新数据并上传
 */
void Upload_Task(void *pvParameters) {
  acquisition_data_t acq_data_local; // 本地数据副本
  TickType_t last_wake_time;

  /* 引用采集模块的全局变量 */
  extern acquisition_data_t g_latest_acquisition_data;
  extern SemaphoreHandle_t g_acquisition_data_mutex;

  /* 初始化时间基准 */
  last_wake_time = xTaskGetTickCount();

  // printf("[Upload] Task started, frequency=%d sec\r\n", g_upload_frequency);

  while (1) {
    /* === OTA期间暂停上传，避免UART4冲突 === */
    extern bool OTA_IsInProgress(void);
    if (OTA_IsInProgress()) {
      // printf("[Upload] OTA in progress, skipping upload\r\n");  //
      // 可选调试日志
      vTaskDelay(pdMS_TO_TICKS(1000)); // 延时1秒后重试
      continue;
    }

    /* 1. 获取互斥锁并拷贝最新采集数据 */
    if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(100)) ==
        pdTRUE) {
      memcpy(&acq_data_local, &g_latest_acquisition_data,
             sizeof(acquisition_data_t));
      xSemaphoreGive(g_acquisition_data_mutex);
    } else {
      printf("[Upload] Failed to acquire mutex, skipping this upload\r\n");
      goto next_cycle;
    }

    /* 2. 格式化JSON数据 */
    Upload_FormatJSON(upload_buffer, UPLOAD_BUFFER_SIZE, &acq_data_local);

    /* 3. 发送数据 */
    Upload_SendData(upload_buffer);

    // printf("[Upload] Data uploaded (freq=%ds)\r\n", g_upload_frequency);

  next_cycle:
    /* 4. 等待下一个上传周期（直接使用全局变量，避免延时计算错误） */
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(g_upload_frequency * 1000));
  }
}

/**
 * @brief  设置上传频率
 */
bool Upload_SetFrequency(uint16_t freq_seconds) {
  if (freq_seconds < UPLOAD_FREQ_MIN || freq_seconds > UPLOAD_FREQ_MAX) {
    return false;
  }

  if (upload_freq_mutex != NULL) {
    if (xSemaphoreTake(upload_freq_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      g_upload_frequency = freq_seconds;
      xSemaphoreGive(upload_freq_mutex);
      return true;
    }
  }

  return false;
}

/**
 * @brief  获取当前上传频率
 */
uint16_t Upload_GetFrequency(void) {
  uint16_t freq = UPLOAD_FREQ_DEFAULT;

  if (upload_freq_mutex != NULL) {
    if (xSemaphoreTake(upload_freq_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      freq = g_upload_frequency;
      xSemaphoreGive(upload_freq_mutex);
    }
  }

  return freq;
}

/**
 * @brief  立即上传一次数据
 */
void Upload_SendNow(void) { upload_now_flag = true; }

/*============================================================================
 *                          私有函数实现
 *===========================================================================*/

/**
 * @brief  格式化JSON数据（参考裸机版本）
 * @param  buffer: 输出缓冲区
 * @param  buffer_size: 缓冲区大小
 * @param  acq_data: 采集数据指针（从队列获取）
 * @note   使用sprintf分步构建，避免Keil编译器转义字符问题
 *         未连接的传感器上传999，便于区分"数据为0"和"未连接"
 */
static void Upload_FormatJSON(char *buffer, uint16_t buffer_size,
                              acquisition_data_t *acq_data) {
  bool gps_valid = false;
  char time_str[30];
  char lat_str[20], lon_str[20];
  char temp_str[10], humi_str[10], voltage_str[10];


  /* 格式化时间字符串 */
  sprintf(time_str, "%04d.%02d.%02d %02d:%02d", acq_data->year, acq_data->month,
          acq_data->day, acq_data->hour, acq_data->min);

  /* 格式化环境数据字符串 */
  sprintf(temp_str, "%.1f", acq_data->env_temp);
  sprintf(humi_str, "%.1f", acq_data->env_humi);
  sprintf(voltage_str, "%d%%", acq_data->battery);

  /* 格式化经纬度字符串 */
  if (acq_data->gps_valid) {
    sprintf(lat_str, "%.6fN", acq_data->latitude);
    sprintf(lon_str, "%.6fE", acq_data->longitude);
  } else {
    sprintf(lat_str, "0.000000N");
    sprintf(lon_str, "0.000000E");
  }

  /* 构建JSON字符串（分步构建，避免转义字符问题） */
  sprintf(buffer, "\r\n{\r\n\"ID\":3,\r\n\"Number\":\"HDY1003\",\r\n\"SN\":"
                  "\"0641100302\",\r\n");
  sprintf(buffer + strlen(buffer),
          "\"Temp\":\"%s\",\r\n\"Humi\":\"%s\",\r\n\"V\":\"%s\",\r\n", temp_str,
          humi_str, voltage_str);
  sprintf(buffer + strlen(buffer),
          "\"Time\":\"%s\",\r\n\"Lat\":\"%s\",\r\n\"Lon\":\"%s\",\r\n",
          time_str, lat_str, lon_str);
  sprintf(buffer + strlen(buffer),
          "\"GPS\":%d,\r\n\"AcqFrequency\":\"%ds\",\r\n\"UploadFrequency\":\"%"
          "ds\",\r\n",
          acq_data->gps_valid ? 1 : 0, acq_data->acq_frequency,
          acq_data->upload_frequency);
  sprintf(buffer + strlen(buffer),
          "\"Channel1\":%d,\r\n\"Channel2\":%d,\r\n\"Channel3\":%d,\r\n",
          WQInterface.Channel[0].connected,
          WQInterface.Channel[1].connected,
          WQInterface.Channel[2].connected);
  sprintf(buffer + strlen(buffer),
          "\"ChannelSensor1\":%d,\r\n\"ChannelSensor2\":%d,"
          "\r\n\"ChannelSensor3\":%d,\r\n",
          (int)WQInterface.Channel[0].type,
          (int)WQInterface.Channel[1].type,
          (int)WQInterface.Channel[2].type);

  /* ⭐ 关键修改：检查COD连接状态，未连接时上传999 */
  if (acq_data->cod_connected) {
    sprintf(buffer + strlen(buffer),
            "\"WTemp\":\"%.2f\",\r\n\"COD\":\"%.6f\",\r\n\"TOC\":\"%.6f\",\r\n",
            acq_data->water_temp, acq_data->cod, acq_data->toc);
    sprintf(buffer + strlen(buffer), "\"TUR\":\"%.6f\",\r\n", acq_data->tur);
  } else {
    sprintf(buffer + strlen(buffer),
            "\"WTemp\":\"999\",\r\n\"COD\":\"999\",\r\n\"TOC\":\"999\","
            "\r\n\"TUR\":\"999\",\r\n");
  }

  /* ⭐ CDOM连接检查 */
  if (acq_data->cdom_connected) {
    sprintf(buffer + strlen(buffer), "\"CDOM\":\"%.6f\",\r\n", acq_data->cdom);
  } else {
    sprintf(buffer + strlen(buffer), "\"CDOM\":\"999\",\r\n");
  }

  /* ⭐ CHL连接检查 */
  if (acq_data->chl_connected) {
    sprintf(buffer + strlen(buffer), "\"CHL\":\"%.6f\",\r\n", acq_data->chl);
  } else {
    sprintf(buffer + strlen(buffer), "\"CHL\":\"999\",\r\n");
  }

  /* ⭐ Y4000连接检查 */
  if (acq_data->y4000_connected) {
    sprintf(buffer + strlen(buffer),
            "\"Y4000DO\":\"%.6f\",\r\n\"Y4000PH\":\"%.6f\",\r\n",
            acq_data->do_val, acq_data->ph);
    sprintf(buffer + strlen(buffer),
            "\"Y4000SAL\":\"%.6f\",\r\n\"Y4000ATM\":\"%.6f\",\r\n",
            acq_data->sal, acq_data->atm);
  } else {
    sprintf(buffer + strlen(buffer),
            "\"Y4000DO\":\"999\",\r\n\"Y4000PH\":\"999\",\r\n\"Y4000SAL\":"
            "\"999\",\r\n\"Y4000ATM\":\"999\",\r\n");
  }

  /* ⭐ 单独传感器连接检查（PH/DO/SAL） */
  if (acq_data->ph_connected) {
    sprintf(buffer + strlen(buffer), "\"PH\":\"%.6f\",\r\n", acq_data->ph);
  } else {
    sprintf(buffer + strlen(buffer), "\"PH\":\"999\",\r\n");
  }

  if (acq_data->do_connected) {
    sprintf(buffer + strlen(buffer), "\"DO\":\"%.6f\",\r\n", acq_data->do_val);
  } else {
    sprintf(buffer + strlen(buffer), "\"DO\":\"999\",\r\n");
  }

  if (acq_data->sal_connected) {
    sprintf(buffer + strlen(buffer), "\"SAL\":\"%.6f\"\r\n}\r\n",
            acq_data->sal);
  } else {
    sprintf(buffer + strlen(buffer), "\"SAL\":\"999\"\r\n}\r\n");
  }
}

/**
 * @brief  发送数据到4G模块（通过UART4）
 * @note   直接操作UART4硬件发送，避免与printf调试输出冲突
 */
static void Upload_SendData(const char *data) {
  if (data == NULL) {
    return;
  }
  if(WQInterface.Network.Send != NULL){
    WQInterface.Network.Send(data);
  }
}
