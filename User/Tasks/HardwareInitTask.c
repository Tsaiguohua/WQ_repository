#include "HardwareInitTask.h"
#include "WQInterface.h"
#include "self_exam.h"
#include "uart4.h"
#include "GPSTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// 硬件初始化任务模块
//
// 功能说明：
//   参照 OV-Watch 的 user_HardwareInitTask.c。
//   本任务以最高优先级启动，确保所有硬件完成初始化后，其他业务任务才能正常采集。
//   初始化完成后自我删除。
//
//   ★ 关键设计：本任务负责"为 WQInterface 插好所有插头"
//     - 固定硬件（AHT20、电量）：通过 WQInterface.XXX.Init() 初始化
//     - 水质传感器（COD/PH/DO等）：通过 SelfExam 自检后绑定到 Channel[]
//
// 作者：蔡国华
// 创建日期：2026/03/18
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  硬件初始化任务（最高优先级，完成后自删除）
 * @note   对标 OV-Watch 的 HardwareInitTask
 */
void HardwareInitTask(void *argument)
{
    uint8_t retry_num;

    /* ⚠️ 挂起调度器：防止业务任务在初始化到一半时抢占 CPU
     *    注意：挂起调度器期间不能调用 vTaskDelay，仅可使用 delay_ms 等忙等待函数
     *    SelfExam_StartExaminate() 内部使用 vTaskDelay，必须在恢复调度器后调用
     */
    vTaskSuspendAll();

    printf("\r\n========== Hardware Initialization Start ==========\r\n");

    /*------------------------------------------------------------------
     * 第一阶段：通信硬件初始化
     *-----------------------------------------------------------------*/

    /* RS485 总线初始化（传感器通信） */
    extern bool rs485_init(void);
    if (rs485_init()) {
        printf("[HWInit] RS485 OK\r\n");
    } else {
        printf("[HWInit] RS485 FAILED!\r\n");
    }

    /*------------------------------------------------------------------
     * 第二阶段：OLED 初始化（⚠️ 必须在任务中初始化，U8g2 堆栈消耗大）
     *-----------------------------------------------------------------*/
#if WQ_USE_Display
    if (WQInterface.Display.Init) {
        WQInterface.Display.Init();
    }
    if (WQInterface.Display.ShowBootAnimation) {
        WQInterface.Display.ShowBootAnimation();  /* 显示开机画面 */
    }
#endif
    printf("[HWInit] OLED OK\r\n");

    /*------------------------------------------------------------------
     * 第三阶段：固定传感器初始化（通过中间层 WQInterface）
     *           对标 OV-Watch 中 HWInterface.AHT21.Init() 的带重试初始化
     *-----------------------------------------------------------------*/

    /* 电量检测初始化 */
    if (WQInterface.Battery.Init != NULL) {
        WQInterface.Battery.Init();
        printf("[HWInit] Battery OK\r\n");
    }

    /* GPS 获取环境数据回调并初始化接收机制 */
    if (WQInterface.GPS.Init != NULL) {
        WQInterface.GPS.Init(GPSTask_RxCallback);
        printf("[HWInit] GPS OK\r\n");
    }

    /* AHT20 温湿度传感器初始化（最多重试3次） */
    retry_num = 3;
    while (retry_num > 0) {
        if (WQInterface.AHT20.Init() == 1) {
            printf("[HWInit] AHT20 OK\r\n");
            break;
        }
        retry_num--;
        if (retry_num == 0) {
            printf("[HWInit] AHT20 FAILED after 3 retries!\r\n");
        }
    }

    /*------------------------------------------------------------------
     * 恢复调度器（在自检之前必须恢复，因为 SelfExam 内部使用 vTaskDelay）
     *-----------------------------------------------------------------*/
    xTaskResumeAll();

    /*------------------------------------------------------------------
     * 第四阶段：水质传感器总线热扫描与通道绑定（★ 核心流程）
     *           必须在恢复调度器之后，因为内部使用 vTaskDelay 等待上电稳定
     *-----------------------------------------------------------------*/

    /* 初始化底层通道控制高侧开关 GPIO */
    SelfExam_Init();

    /* 启动中间层的传感器扫描及函数挂载 */
    if (WQInterface.System.ScanAndBindChannels) {
        WQInterface.System.ScanAndBindChannels();
    }
    printf("[HWInit] Central Sensor Scanning & Binding Done\r\n");

    /* 挂载 TF 卡本地文件系统  */

    if (WQInterface.Storage.Init ) {
        WQInterface.Storage.Init();
    }

    /*------------------------------------------------------------------
     * 收尾：打印完成信息，任务自删除
     *-----------------------------------------------------------------*/
    printf("========== Hardware Initialization Done ==========\r\n\r\n");

    vTaskDelete(NULL);  /* 任务使命完成，自我删除，释放内存 */
}
