#include "TasksInit.h"
#include "HardwareInitTask.h"
#include "AcqTask.h"
#include "UpLoadTask.h"
#include "TFTask.h"
#include "uart4.h"
#include "CommandTask.h"
#include "OLEDTask.h"
#include "GPSTask.h"
#include "HeartBeatTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cJSON.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// 任务统一初始化模块
//
// 功能说明：
//   参照 OV-Watch 的 user_TasksInit.c，统一在此处：
//   1. 定义所有任务句柄和属性
//   2. 创建任务间通信的队列和信号量
//   3. 创建所有业务任务
//
//   ★ HardwareInitTask 优先级最高，保证硬件初始化完成后其他任务才能运行
//   ★ HardwareInitTask 完成后自删除，释放内存
//
// 作者：蔡国华
// 创建日期：2026/03/18
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          任务优先级配置
 *===========================================================================*/

#define PRIO_HARDWARE_INIT  9   // 最高优先级，确保硬件先初始化
#define PRIO_CMD_TASK       7   // 命令任务（MQTT指令处理，响应及时）
#define PRIO_ACQ_TASK       6   // 采集任务
#define PRIO_APP_TASK       5   // 保留（如需单独任务处理环境数据）
#define PRIO_OLED_TASK      4   // OLED 刷新
#define PRIO_GPS_TASK       4   // GPS 解析
#define PRIO_UPLOAD_TASK    3   // 上传任务
#define PRIO_TF_TASK        3   // TF卡存储任务
#define PRIO_HEARTBEAT_TASK 2   // 心跳保活
#define PRIO_WATCHDOG_TASK  1   // 看门狗

/*============================================================================
 *                          任务堆栈配置
 *===========================================================================*/

#define STK_HARDWARE_INIT  768  // 硬件初始化任务（OLED_Init 需要大堆栈）
#define STK_ACQ_TASK       640  // 采集任务
#define STK_UPLOAD_TASK    768  // 上传任务（JSON格式化需要）
#define STK_TF_TASK        1024 // TF卡写入（FatFs与大乱写需要大堆栈）
#define STK_CMD_TASK       640  // 命令任务
#define STK_OLED_TASK      640  // OLED 刷新
#define STK_GPS_TASK       384  // GPS 解析
#define STK_HEARTBEAT_TASK 512  // 心跳任务
#define STK_OTA_TASK       512  // OTA 任务

/*============================================================================
 *                          任务句柄定义（对外暴露，供 OTA 模块使用）
 *===========================================================================*/

TaskHandle_t AcqTaskHandle       = NULL;
TaskHandle_t UploadTaskHandle    = NULL;
TaskHandle_t TFTaskHandle        = NULL;
TaskHandle_t OledTaskHandle      = NULL;
TaskHandle_t CmdTaskHandle       = NULL;
TaskHandle_t GpsTaskHandle       = NULL;
TaskHandle_t HeartbeatTaskHandle = NULL;

/*============================================================================
 *                          队列/信号量句柄定义
 *===========================================================================*/

/* UART4 接收队列（双缓冲机制） */
QueueHandle_t xUartRxQueue = NULL;

/* 系统事件组（用于任务间同步，替代全局标志变量） */
EventGroupHandle_t g_system_events = NULL;

/*============================================================================
 *                          统一任务初始化入口
 *===========================================================================*/

/**
 * @brief  统一任务初始化函数（在 main() 中调用）
 * @note   对标 OV-Watch 的 User_Tasks_Init()
 *         - 创建任务间通信的队列和信号量
 *         - 创建 HardwareInitTask（优先级最高，负责按顺序初始化所有硬件）
 *         - 创建所有业务任务（采集、上传、OLED、命令等）
 */
void User_Tasks_Init(void) {
 
    printf("\r\n========== Creating FreeRTOS Tasks ==========\r\n");

    /* 初始化 cJSON 内存钩子：让 cJSON 强制使用 FreeRTOS 那 80KB 的超大堆 */
    cJSON_Hooks hooks;
    hooks.malloc_fn = pvPortMalloc;
    hooks.free_fn = vPortFree;
    cJSON_InitHooks(&hooks);

    /*------------------------------------------------------------------
     * 第一步：创建任务间通信对象
     *-----------------------------------------------------------------*/

    /* UART4 接收消息队列（容量5条，每条 UartRxMsg_t 大小） */
    xUartRxQueue = xQueueCreate(5, sizeof(UartRxMsg_t));
    if (xUartRxQueue == NULL) {
        printf("[TasksInit] ERROR: xUartRxQueue create failed!\r\n");
    }

    /* 系统事件组（用于 HardwareInitTask → 业务任务 的同步） */
    g_system_events = xEventGroupCreate();
    if (g_system_events == NULL) {
        printf("[TasksInit] ERROR: g_system_events create failed!\r\n");
    }

    /* GPS 同步量与互斥锁预先创建，防止 IRQ 在此时到达导致错误 */
    GPS_TaskInit();

    /*------------------------------------------------------------------
     * 第二步：创建 HardwareInitTask（最高优先级，初始化后自删除）
     *         对标 OV-Watch 的 HardwareInitTask
     *-----------------------------------------------------------------*/
    xTaskCreate(HardwareInitTask,   /* 任务函数 */
                "HWInitTask",       /* 任务名 */
                STK_HARDWARE_INIT,  /* 堆栈大小（字） */
                NULL,               /* 参数 */
                PRIO_HARDWARE_INIT, /* 最高优先级 */
                NULL);              /* 句柄（初始化任务自删，不需要句柄） */

    /*------------------------------------------------------------------
     * 第三步：创建所有业务任务
     *-----------------------------------------------------------------*/

    /* 采集任务（水质传感器 + GPS + 环境数据） */
    xTaskCreate(Acquisition_Task,
                "AcqTask",
                STK_ACQ_TASK,
                NULL,
                PRIO_ACQ_TASK,
                &AcqTaskHandle);

    /* 数据上传任务（MQTT JSON 上传） */
    xTaskCreate(Upload_Task,
                "UploadTask",
                STK_UPLOAD_TASK,
                NULL,
                PRIO_UPLOAD_TASK,
                &UploadTaskHandle);

    /* TF 卡本地存储任务 (CSV 和 TXT 双格式写入) */
    xTaskCreate(TF_Task,
                "TFTask",
                STK_TF_TASK,
                NULL,
                PRIO_TF_TASK,
                &TFTaskHandle);

    /* OLED 显示任务 */
    xTaskCreate(OledTask,
                "OledTask",
                STK_OLED_TASK,
                NULL,
                PRIO_OLED_TASK,
                &OledTaskHandle);

    xTaskCreate(Command_Task,
                "CmdTask",
    /* MQTT 命令解析任务 */
                STK_CMD_TASK,
                NULL,
                PRIO_CMD_TASK,
                &CmdTaskHandle);

    /* OTA 任务（始终存在，等待升级指令） */
    extern void OTA_TaskInit(void);
    OTA_TaskInit();

    /* GPS 数据接收与解析任务 */
    xTaskCreate(GPS_Task,
                "GPSTask",
                STK_GPS_TASK,
                NULL,
                PRIO_GPS_TASK,
                &GpsTaskHandle);

    /* MQTT 设备保活心跳任务 */
    xTaskCreate(Heartbeat_Task,
                "Heartbeat",
                STK_HEARTBEAT_TASK,
                NULL,
                PRIO_HEARTBEAT_TASK,
                &HeartbeatTaskHandle);

    /* ⚠️ 注册任务句柄到 OTA 模块（OTA 期间需暂停/恢复这些任务） */
    extern void OTA_RegisterTasks(TaskHandle_t acq, TaskHandle_t upload,
                                  TaskHandle_t app, TaskHandle_t oled,
                                  TaskHandle_t heartbeat, TaskHandle_t watchdog);
    OTA_RegisterTasks(AcqTaskHandle, UploadTaskHandle, NULL,
                      OledTaskHandle, HeartbeatTaskHandle, NULL);

    printf("[TasksInit] All tasks created.\r\n");
    printf("==============================================\r\n\r\n");
}
