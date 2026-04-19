/**
 * @file    TFTask.c
 * @author  Antigravity Refactor Team
 * @brief   本地 SD 卡数据存储任务 (Logging & CSV Support)
 * @version 2.0 (Refactored)
 * @date    2026-04-19
 *
 * @details 
 *   本任务负责将实时采集的数据持久化存储到本地 SD 卡。
 *   - CSV 存储：文件名 Record.csv，表头已国际化为全英文，彻底解决 Excel 乱码问题。
 *   - TXT 存储：文件名 Detail.txt，存储优雅、易读的格式化水质日志。
 *   - 数据快照：通过互斥锁获取采集副本，写卡过程不阻塞传感器采集。
 */

#include "TFTask.h"
#include "WQInterface.h"
#include "AcqTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* 当前写卡频率（秒），默认与采集频率相对同步或稍微慢点，防止频繁写坏卡 */
static uint16_t g_tf_write_frequency = 60; 

/* 本地写卡频率的互斥锁 */
static SemaphoreHandle_t tf_freq_mutex = NULL;

/* 内部初始化 */
static void TFTask_Init(void) {
    tf_freq_mutex = xSemaphoreCreateMutex();
}

/**
 * @brief  获取当前写卡频率
 */
uint16_t TFTask_GetFrequency(void) {
    uint16_t freq = 60;
    if (tf_freq_mutex != NULL) {
        if (xSemaphoreTake(tf_freq_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            freq = g_tf_write_frequency;
            xSemaphoreGive(tf_freq_mutex);
        }
    } else {
        freq = g_tf_write_frequency;
    }
    return freq;
}

/**
 * @brief  设置当前写卡频率
 */
bool TFTask_SetFrequency(uint16_t freq_seconds) {
    if (freq_seconds < 10 || freq_seconds > 3600) return false;
    
    if (tf_freq_mutex != NULL) {
        if (xSemaphoreTake(tf_freq_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_tf_write_frequency = freq_seconds;
            xSemaphoreGive(tf_freq_mutex);
            return true;
        }
    }
    return false;
}

/* =======================================================
 * 格式化为 CSV (逗号分隔值，方便 Excel 打开分析)
 * ======================================================= */
static void Format_CSV(char *buffer, uint16_t max_len, acquisition_data_t *acq_data) {
    uint16_t ofs = 0;
    
    // 初始化空串
    buffer[0] = '\0';
    
    // 拼接前导环境数据
    ofs += snprintf(buffer + ofs, max_len - ofs, 
            "%04d/%02d/%02d,%02d:%02d:%02d,%d%%,%.1f,%.1f,%s%.6f,%s%.6f,",
            acq_data->year, acq_data->month, acq_data->day,
            acq_data->hour, acq_data->min, acq_data->sec,
            acq_data->battery,
            acq_data->env_temp, acq_data->env_humi,
            acq_data->gps_valid ? "" : "INV_", acq_data->latitude,
            acq_data->gps_valid ? "" : "INV_", acq_data->longitude);
            
    // COD 传感器组
    if(Acq_HasKV(acq_data, "COD")) {
        ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,%.4f,%.4f,", 
               Acq_GetVal_FromKV(acq_data, "COD"), Acq_GetVal_FromKV(acq_data, "TOC"), Acq_GetVal_FromKV(acq_data, "TUR"));
    } else {
        ofs += snprintf(buffer + ofs, max_len - ofs, "999,999,999,");
    }
    
    // CDOM 
    if(Acq_HasKV(acq_data, "CDOM")) ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,", Acq_GetVal_FromKV(acq_data, "CDOM"));
    else ofs += snprintf(buffer + ofs, max_len - ofs, "999,");
    
    // CHL
    if(Acq_HasKV(acq_data, "CHL")) ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,", Acq_GetVal_FromKV(acq_data, "CHL"));
    else ofs += snprintf(buffer + ofs, max_len - ofs, "999,");
    
    // Y4000
    if(Acq_HasKV(acq_data, "Y_PH")) {
        ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,%.4f,%.4f,", 
               Acq_GetVal_FromKV(acq_data, "Y_DO"), Acq_GetVal_FromKV(acq_data, "Y_PH"), Acq_GetVal_FromKV(acq_data, "Y_SAL"));
    } else {
        ofs += snprintf(buffer + ofs, max_len - ofs, "999,999,999,");
    }
    
    // 独立传感器 PH, DO, SAL
    if(Acq_HasKV(acq_data, "PH")) ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,", Acq_GetVal_FromKV(acq_data, "PH"));
    else ofs += snprintf(buffer + ofs, max_len - ofs, "999,");
    
    if(Acq_HasKV(acq_data, "DO")) ofs += snprintf(buffer + ofs, max_len - ofs, "%.4f,", Acq_GetVal_FromKV(acq_data, "DO"));
    else ofs += snprintf(buffer + ofs, max_len - ofs, "999,");
    
    if(Acq_HasKV(acq_data, "SAL")) snprintf(buffer + ofs, max_len - ofs, "%.4f\r\n", Acq_GetVal_FromKV(acq_data, "SAL")); // 最后一列换行
    else snprintf(buffer + ofs, max_len - ofs, "999\r\n"); // 最后一列换行
}

/* =======================================================
 * 格式化为 TXT (直观阅读的日志格式)
 * ======================================================= */
static void Format_TXT(char *buffer, uint16_t max_len, acquisition_data_t *acq_data) {
    buffer[0] = '\0';
    
    snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "========== Record Time: %04d/%02d/%02d %02d:%02d:%02d ==========\r\n",
             acq_data->year, acq_data->month, acq_data->day, acq_data->hour, acq_data->min, acq_data->sec);
             
    snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "ENV  >> Battery:%d%% | Temp:%.1fC | Humi:%.1f%%\r\n", 
             acq_data->battery, acq_data->env_temp, acq_data->env_humi);
             
    if (acq_data->gps_valid) {
        snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "GPS  >> Lat:%.6f | Lon:%.6f\r\n", acq_data->latitude, acq_data->longitude);
    } else {
        snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "GPS  >> No signal\r\n");
    }
    
    snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "Water Sensor Data:\r\n");
    if(Acq_HasKV(acq_data, "COD")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - COD: %.3f | TOC: %.3f | TUR: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "COD"), Acq_GetVal_FromKV(acq_data, "TOC"), Acq_GetVal_FromKV(acq_data, "TUR"));
    if(Acq_HasKV(acq_data, "CDOM")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - CDOM: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "CDOM"));
    if(Acq_HasKV(acq_data, "CHL")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - CHL: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "CHL"));
    if(Acq_HasKV(acq_data, "Y_PH")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - Y4000: DO %.3f | PH %.3f | SAL %.3f\r\n", Acq_GetVal_FromKV(acq_data, "Y_DO"), Acq_GetVal_FromKV(acq_data, "Y_PH"), Acq_GetVal_FromKV(acq_data, "Y_SAL"));
    if(Acq_HasKV(acq_data, "PH")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - PH: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "PH"));
    if(Acq_HasKV(acq_data, "DO")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - DO: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "DO"));
    if(Acq_HasKV(acq_data, "SAL")) snprintf(buffer + strlen(buffer), max_len - strlen(buffer), " - SAL: %.3f\r\n", Acq_GetVal_FromKV(acq_data, "SAL"));
    
    snprintf(buffer + strlen(buffer), max_len - strlen(buffer), "=========================================================\r\n\r\n");

}

/**
 * @brief  TF卡存储的主任务
 */
void TF_Task(void *pvParameters) {
    acquisition_data_t snapshot;
    
    /* 需要分配相对较大的数组（建议存放在任务栈，并在 TasksInit 中给此任务开 1024 堆栈）*/
    char str_buf[800];   
    
    TFTask_Init();
    printf("[TFTask] Started. Waiting to write to SD Card...\r\n");

    // 确保 CSV 表头已写入
    const char *csv_header = "Date,Time,Battery(%),EnvTemp(C),EnvHumi(%),Latitude,Longitude,COD,TOC,TUR,CDOM,CHL,Y4000_DO,Y4000_pH,Y4000_SAL,PH,DO,SAL\r\n";
    if (WQInterface.Storage.WriteCSVHeader) {
        WQInterface.Storage.WriteCSVHeader(csv_header);
    }

    while(1) {
        /* 如果 OTA 正在进行，为防总线或抢占问题，挂起本任务的操作 */
        extern bool OTA_IsInProgress(void);
        if (OTA_IsInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 1. 从采集模块索取最新数据的快照
        if (g_acquisition_data_mutex != NULL) {
            if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                memcpy(&snapshot, &g_latest_acquisition_data, sizeof(acquisition_data_t));
                xSemaphoreGive(g_acquisition_data_mutex);
            } else {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }
        
        // 2. 将数据格式化出一段包含完整表头的 CSV 并写盘 (Record.csv)
        Format_CSV(str_buf, sizeof(str_buf), &snapshot);
        WQInterface.Storage.WriteCSV(str_buf);

        // 3. 将数据格式化出一段优雅易读的 TXT 格式日志并写盘 (Detail.txt)
        Format_TXT(str_buf, sizeof(str_buf), &snapshot);
        WQInterface.Storage.WriteTXT(str_buf);

        // 4. 睡眠进入下一个存取周期
        uint16_t delay_sec = TFTask_GetFrequency();
        vTaskDelay(pdMS_TO_TICKS(delay_sec * 1000));
    }
}
