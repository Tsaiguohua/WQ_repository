#include "UpLoadTask.h"
#include "WQInterface.h"
#include "AcqTask.h"      
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cJSON.h" 
#include <stdio.h>
#include <string.h>

/* 当前上传频率（秒） */
uint16_t g_upload_frequency = 10; // 默认30秒上传一次



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
        if (WQInterface.Network.connected) {
            WQInterface.Network.Send(json_buf);
        }
        
        // 4. 等待上传周期
        uint16_t delay_sec = Upload_GetFrequency();
        vTaskDelay(pdMS_TO_TICKS(delay_sec * 1000));
    }
}
    

/**
 * @brief  格式化JSON数据（使用 cJSON 层层构建）
 * @param  buffer: 输出缓冲区
 * @param  buffer_size: 缓冲区大小
 * @param  acq_data: 采集数据指针
 */
static void Upload_FormatJSON(char *buffer, uint16_t buffer_size,
                              acquisition_data_t *acq_data) {
  /* 1. 创建 cJSON 根节点 */
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
      buffer[0] = '\0';
      return; 
  }

  char time_str[30], lat_str[20], lon_str[20];
  char temp_str[10], humi_str[10], voltage_str[10];
  char freq_str[10], val_str[20];

  /* 基础格式化 */
  sprintf(time_str, "%04d.%02d.%02d %02d:%02d", acq_data->year, acq_data->month,
          acq_data->day, acq_data->hour, acq_data->min);
  
  if (acq_data->gps_valid) {
    sprintf(lat_str, "%.6fN", acq_data->latitude);
    sprintf(lon_str, "%.6fE", acq_data->longitude);
  } else {
    sprintf(lat_str, "0.000000N");
    sprintf(lon_str, "0.000000E");
  }

  sprintf(temp_str, "%.1f", acq_data->env_temp);
  sprintf(humi_str, "%.1f", acq_data->env_humi);
  sprintf(voltage_str, "%d%%", acq_data->battery);

  /* 2. 填充系统基础数据 */
  cJSON_AddNumberToObject(root, "ID", 3);
  cJSON_AddStringToObject(root, "Number", "HDY1003");
  cJSON_AddStringToObject(root, "SN", "0641100302");
  cJSON_AddStringToObject(root, "Temp", temp_str);
  cJSON_AddStringToObject(root, "Humi", humi_str);
  cJSON_AddStringToObject(root, "V", voltage_str);
  cJSON_AddStringToObject(root, "Time", time_str);
  cJSON_AddStringToObject(root, "Lat", lat_str);
  cJSON_AddStringToObject(root, "Lon", lon_str);
  cJSON_AddNumberToObject(root, "GPS", acq_data->gps_valid ? 1 : 0);

  sprintf(freq_str, "%ds", acq_data->acq_frequency);
  cJSON_AddStringToObject(root, "AcqFrequency", freq_str);
  
  sprintf(freq_str, "%ds", acq_data->upload_frequency);
  cJSON_AddStringToObject(root, "UploadFrequency", freq_str);

  cJSON_AddNumberToObject(root, "Channel1", WQInterface.Channel[0].connected);
  cJSON_AddNumberToObject(root, "Channel2", WQInterface.Channel[1].connected);
  cJSON_AddNumberToObject(root, "Channel3", WQInterface.Channel[2].connected);
  cJSON_AddNumberToObject(root, "ChannelSensor1", (int)WQInterface.Channel[0].type);
  cJSON_AddNumberToObject(root, "ChannelSensor2", (int)WQInterface.Channel[1].type);
  cJSON_AddNumberToObject(root, "ChannelSensor3", (int)WQInterface.Channel[2].type);

  /* 3. 填充 COD 参数 */
  if (Acq_HasKV(acq_data, "COD")) {
    sprintf(val_str, "%.2f", Acq_GetVal_FromKV(acq_data, "WTemp")); cJSON_AddStringToObject(root, "WTemp", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "COD"));   cJSON_AddStringToObject(root, "COD", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "TOC"));   cJSON_AddStringToObject(root, "TOC", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "TUR"));   cJSON_AddStringToObject(root, "TUR", val_str);
  } else {
    cJSON_AddStringToObject(root, "WTemp", "999");
    cJSON_AddStringToObject(root, "COD", "999");
    cJSON_AddStringToObject(root, "TOC", "999");
    cJSON_AddStringToObject(root, "TUR", "999");
  }

  /* 4. 填充单参数传感器 */
  if (Acq_HasKV(acq_data, "CDOM")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "CDOM")); cJSON_AddStringToObject(root, "CDOM", val_str);
  } else { cJSON_AddStringToObject(root, "CDOM", "999"); }

  if (Acq_HasKV(acq_data, "CHL")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "CHL")); cJSON_AddStringToObject(root, "CHL", val_str);
  } else { cJSON_AddStringToObject(root, "CHL", "999"); }

  /* 5. 填充 Y4000 参数 */
  if (Acq_HasKV(acq_data, "Y_PH")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "Y_DO"));  cJSON_AddStringToObject(root, "Y4000DO", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "Y_PH"));  cJSON_AddStringToObject(root, "Y4000PH", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "Y_SAL")); cJSON_AddStringToObject(root, "Y4000SAL", val_str);
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "Y_ATM")); cJSON_AddStringToObject(root, "Y4000ATM", val_str);
  } else {
    cJSON_AddStringToObject(root, "Y4000DO", "999");
    cJSON_AddStringToObject(root, "Y4000PH", "999");
    cJSON_AddStringToObject(root, "Y4000SAL", "999");
    cJSON_AddStringToObject(root, "Y4000ATM", "999");
  }

  if (Acq_HasKV(acq_data, "PH")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "PH")); cJSON_AddStringToObject(root, "PH", val_str);
  } else { cJSON_AddStringToObject(root, "PH", "999"); }

  if (Acq_HasKV(acq_data, "DO")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "DO")); cJSON_AddStringToObject(root, "DO", val_str);
  } else { cJSON_AddStringToObject(root, "DO", "999"); }

  if (Acq_HasKV(acq_data, "SAL")) {
    sprintf(val_str, "%.6f", Acq_GetVal_FromKV(acq_data, "SAL")); cJSON_AddStringToObject(root, "SAL", val_str);
  } else { cJSON_AddStringToObject(root, "SAL", "999"); }

  /* 6. 生成无缩进的 JSON 字符串 */
  char *json_out = cJSON_PrintUnformatted(root); 
  if (json_out != NULL) {
    /* 为了既保留 cJSON 的安全性，又满足你需要的"竖向单行且无缩进"的格式，做一次极简替换：
       碰到 '{' 或 ',' 时就在后面加上 \r\n，碰到 '}' 时在前面加上 \r\n */
    int out_index = 0;
    
    /* 配合协议，在最开头加个回车 */
    buffer[out_index++] = '\r';
    buffer[out_index++] = '\n';

    for (int i = 0; json_out[i] != '\0'; i++) {
        /* 如果这是 JSON 结尾的括号，先换个行再打括号 */
        if (json_out[i] == '}') {
            buffer[out_index++] = '\r';
            buffer[out_index++] = '\n';
        }

        /* 正常拷贝字符 */
        buffer[out_index++] = json_out[i];
        
        /* 如果这是大括号或者逗号结尾，直接加换行 */
        if (json_out[i] == '{' || json_out[i] == ',') {
            buffer[out_index++] = '\r';
            buffer[out_index++] = '\n';
        }
    }
    
    /* 结尾配合协议再补一个回车 */
    buffer[out_index++] = '\r';
    buffer[out_index++] = '\n';
    buffer[out_index] = '\0';

    /* 必须 free 掉 cJSON 吐出来的内存 */
    cJSON_free(json_out); 
  } else {
    buffer[0] = '\0';
  }
  
  /* 必须删掉整个树，释放掉所有对象的树节点内存 */
  cJSON_Delete(root);
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
