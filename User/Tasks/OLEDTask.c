#include "OLEDTask.h"
#include "WQInterface.h"
#include "AcqTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdbool.h>

void OledTask(void *pvParameters) {
    char battery_str[10];
    char time_str[10];
    char year_str[15];
    char temp_str[10];
    char freq_str[10];
    char hum_str[10];

    while (1) {

        /* 收集刷新主界面依赖的数据 */
        uint8_t battery = 0;

        if (WQInterface.Battery.GetBattery) {
            battery = WQInterface.Battery.GetBattery();
        }


        float temp = 25.0f;
        float humi = 60.0f;
        uint16_t upload_freq = 1;

        if (g_acquisition_data_mutex != NULL &&
            xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            temp = g_latest_acquisition_data.env_temp;
            humi = g_latest_acquisition_data.env_humi;
            upload_freq = g_latest_acquisition_data.upload_frequency;
            xSemaphoreGive(g_acquisition_data_mutex);
        }

        /* 格式化显示字符串 */
        sprintf(battery_str, "%d%%", battery);
        sprintf(time_str, "05:04"); // 临时固定时间
        sprintf(year_str, "2025-12-31");
        sprintf(temp_str, "%.0fC", temp);
        sprintf(freq_str, "%ds", upload_freq);
        sprintf(hum_str, "%.0f%%", humi);


        if (WQInterface.Display.ShowMainScreen) {
            WQInterface.Display.ShowMainScreen(
                year_str, time_str, battery_str, temp_str, freq_str, hum_str,
                WQInterface.Channel[0].connected, WQInterface.Channel[1].connected, WQInterface.Channel[2].connected
            );
        }


        /* 1 秒刷新周期 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
