#include "UpLoadTask.h"
#include "WQInterface.h"
#include "AcqTask.h"      
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* 当前上传频率（秒） */
uint16_t g_upload_frequency = 30; // 默认30秒上传一次

/* 立即上传标志 */
static volatile bool upload_now_flag = false;

/* 互斥量（保护上传频率） */
static SemaphoreHandle_t upload_freq_mutex = NULL;

/* 内部格式化函数声明 */
static void Upload_FormatJSON(char *buffer, uint16_t buffer_size,
                              acquisition_data_t *acq_data);

typedef struct {
    float env_temp;
    float env_humi;
    uint8_t battery;
    
    float latitude;
    float longitude;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    bool gps_valid;
    
    uint16_t acq_frequency;
    uint16_t upload_frequency;
    
    // 省略：为避免未定义，最好引回 acquisition.h 或重新包含相关的定义头。
} UploadTemp_t; // 本作兼容

void Upload_Init(void) {
  /* 创建互斥量 */
  upload_freq_mutex = xSemaphoreCreateMutex();
}


//立即上传一次数据
void Upload_SendNow(void) {
  upload_now_flag = true; 
}

void Upload_Task(void *pvParameters) {
    acquisition_data_t snapshot;
    char json_buf[1200];
    
    Upload_Init();

    while (1) {
        /* === OTA期间暂停上传，避免UART冲突 === */
        extern bool OTA_IsInProgress(void);
        if (OTA_IsInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 1. 加锁，拷贝采集数据
        if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            memcpy(&snapshot, &g_latest_acquisition_data, sizeof(acquisition_data_t));
            xSemaphoreGive(g_acquisition_data_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 2. 格式化 JSON（使用精细判断版本）
        Upload_FormatJSON(json_buf, sizeof(json_buf), &snapshot);
        printf("\r\n准备存入sd卡的数据:\r\n%s\r\n",json_buf);
	   
        // 3. 发送（通过中间层）
    //    if (WQInterface.Network.connected) {
    //        WQInterface.Network.Send(json_buf);
    //    }
        
        // 4. 等待上传周期
        uint16_t delay_sec = Upload_GetFrequency();
        vTaskDelay(pdMS_TO_TICKS(delay_sec * 1000));
    }
}
    

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
          acq_data->channel1_state, acq_data->channel2_state,
          acq_data->channel3_state);
  sprintf(buffer + strlen(buffer),
          "\"ChannelSensor1\":%d,\r\n\"ChannelSensor2\":%d,"
          "\r\n\"ChannelSensor3\":%d,\r\n",
          acq_data->channel1_sensor, acq_data->channel2_sensor,
          acq_data->channel3_sensor);

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
 * @brief  获取当前上传频率
 */
uint16_t Upload_GetFrequency(void) {
  uint16_t freq = 30;

  if (upload_freq_mutex != NULL) {
    if (xSemaphoreTake(upload_freq_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      freq = g_upload_frequency;
      xSemaphoreGive(upload_freq_mutex);
    }
  } else {
    freq = g_upload_frequency;
  }

  return freq;
}

/**
 * @brief  设置上传频率
 */
bool Upload_SetFrequency(uint16_t freq_seconds) {
  if (freq_seconds < 10 || freq_seconds > 3600) {
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
