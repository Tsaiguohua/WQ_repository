#include "GPSTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "WQInterface.h"

/* FreeRTOS 同步对象 */
static SemaphoreHandle_t s_gps_frame_sem = NULL;
static SemaphoreHandle_t s_gps_data_mutex = NULL;

/* 全局缓存的 GPS 解析数据 */
static gps_data_t s_gps_task_data = {0};
static bool s_has_valid_data = false;

/**
 * @brief 驱动层的接收中断回调，当 NMEA 帧组装结束时触发，由外部硬件驱动层唤醒
 */
void GPSTask_RxCallback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_gps_frame_sem != NULL) {
        xSemaphoreGiveFromISR(s_gps_frame_sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 创建 GPS 解析任务需要的锁和信号量
 */
void GPS_TaskInit(void) {
    /* 创建二值信号量 */
    s_gps_frame_sem = xSemaphoreCreateBinary();
    if (s_gps_frame_sem == NULL) {
        while (1);
    }

    /* 创建互斥锁 */
    s_gps_data_mutex = xSemaphoreCreateMutex();
    if (s_gps_data_mutex == NULL) {
        while (1);
    }
}

/**
 * @brief FreeRTOS 解析任务
 */
void GPS_Task(void *pvParameters) {
    char frame_buf[100];
    gps_data_t temp_parsed_data;

    while (1) {
        /* 等待驱动层 NMEA 帧接收完毕的信号量 */
        if (xSemaphoreTake(s_gps_frame_sem, portMAX_DELAY) == pdTRUE) {
            
            /* 通过中间层提取原始缓冲 */
            if (WQInterface.GPS.GetRawFrame(frame_buf, sizeof(frame_buf))) {
                
                /* 通过中间层解析数据 */
                if (WQInterface.GPS.ParseNMEA(frame_buf, &temp_parsed_data)) {
                    
                    /* 解析成功，通过互斥锁更新全局数据 */
                    if (xSemaphoreTake(s_gps_data_mutex, portMAX_DELAY) == pdTRUE) {
                        s_gps_task_data = temp_parsed_data;
                        s_has_valid_data = true;
                        xSemaphoreGive(s_gps_data_mutex);
                    }
                }
            }
        }
    }
}

bool GPSTask_GetData(gps_data_t *data) {
    bool ret = false;

    if (s_gps_data_mutex == NULL || !s_has_valid_data) {
        return false;
    }

    if (xSemaphoreTake(s_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_gps_task_data.is_valid) {
            *data = s_gps_task_data;
            ret = true;
        }
        xSemaphoreGive(s_gps_data_mutex);
    }

    return ret;
}
