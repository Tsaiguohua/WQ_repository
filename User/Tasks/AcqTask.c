#include "AcqTask.h"
#include "TasksInit.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"

/* 移除底层传感器驱动和硬件相关的头文件，纯净地通过 WQInterface 调用 */
#include "gps.h"       // 仅因为内部还需要用到 gps_data_t

/* 自检模块（获取自动识别的传感器类型,并控制通道GPIO） */
#include "self_exam.h"

/* 上传模块（获取上传频率） */
#include "UpLoadTask.h"

/* 中间层（通过 WQInterface 调用所有硬件） */
#include "WQInterface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
// 数据采集任务模块 - FreeRTOS版本
//
// 功能说明：
//   本模块负责周期性采集所有传感器数据，包括GPS、环境数据（温湿度、电量）
//   和水质传感器数据（COD、CDOM、CHL、PH、DO、SAL、Y4000等）
//
// 传感器自动识别机制：
//   1. 系统启动时，self_exam模块自动识别每个通道的传感器类型
//   2. 采集任务根据识别结果调用相应的传感器驱动
//   3. 支持即插即用，无需手动配置
//
// 数据流向：
//   采集任务 → g_latest_acquisition_data（全局共享） → 上传任务/OLED任务
//   ├─ GPS数据（经纬度、时间）
//   ├─ 环境数据（AHT20温湿度、电池电量）
//   └─ 水质数据（根据通道类型采集）
//
// 频率控制：
//   - 默认采集频率：5秒
//   - 可通过MQTT命令动态修改：{"AcqFrequency": 15}
//   - 修改后自动保存到Flash，重启后恢复
//   - 范围：1~3600秒
//
// 通道管理：
//   - 3个独立通道，每个可连接不同类型的水质传感器
//   - 通道状态（开/关）通过GPIO继电器控制
//   - 开启通道时延时2秒等待继电器稳定
//
// 多任务保护：
//   - 使用互斥锁（g_acquisition_data_mutex）保护共享数据
//   - 采集任务写入数据前获取锁
//   - 上传任务/OLED任务读取数据前也需获取锁
//
// 使用示例：
//   1. 启动时自动创建采集任务（app_init_task中）
//   2. 任务自动运行，按采集频率采集数据
//   3. 其他任务通过互斥锁读取g_latest_acquisition_data
//
// 注意事项：
//   ⚠️ 通道GPIO控制由self_exam模块提供，不要直接操作GPIO
//   ⚠️ 读取共享数据时必须使用互斥锁保护
//   ⚠️ 采集频率不应过快，传感器需要测量时间
//
// 作者：蔡国华
// 创建日期：2026/01/03
// 最后更新：2026/01/11
// 版本：v1.0
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          全局变量定义
 *===========================================================================*/

/* 采集频率（秒） */
uint16_t g_acquisition_frequency = ACQ_FREQ_DEFAULT;

/* 移除所有独立声明的 onechannel_state 等变量，
 * 现已完全被中间层 WQInterface 所挂管的 Channel[i].connected 及 Channel[i].type 代替。
 */

/* 最新采集数据（全局共享数据） */
acquisition_data_t g_latest_acquisition_data;

/* 保护采集数据的互斥锁 */
SemaphoreHandle_t g_acquisition_data_mutex = NULL;

/*============================================================================
 *                          内部函数声明
 *===========================================================================*/

static void collect_gps_data(acquisition_data_t *data);
static void collect_environment_data(acquisition_data_t *data);
static void collect_all_water_sensors(acquisition_data_t *data);

/*============================================================================
 *                          采集模块初始化
 *===========================================================================*/

/**
 * @brief  初始化采集模块
 */
void Acquisition_Init(void) {
  /* 创建互斥锁保护共享数据 */
  g_acquisition_data_mutex = xSemaphoreCreateMutex();
  if (g_acquisition_data_mutex == NULL) {
    printf("[Acquisition] ERROR: Failed to create mutex!\r\n");
  }

  // printf("[Acquisition] Init done, frequency=%d sec\r\n",
 //        g_acquisition_frequency);
}

/*============================================================================
 *                          采集任务主函数
 *===========================================================================*/

/**
 * @brief  采集任务函数
 */
void Acquisition_Task(void *pvParameters) {
  acquisition_data_t data;
  TickType_t last_wake_time;
  uint32_t delay_ticks;

  /* 初始化 */
  Acquisition_Init();

  /*------------------------------------------------------------------
   * ★ 关键同步点：等待 HardwareInitTask 完成传感器自检 + 绑定
   *    使用事件组（Event Group）替代全局标志变量，零时序竞争。
   *    在灯亮之前，本任务挂起休眠，不消耗任何 CPU 时间。
   *-----------------------------------------------------------------*/
  printf("[Acq] Waiting for hardware init to complete...\r\n");
  xEventGroupWaitBits(g_system_events,   /* 事件组句柄 */
                      EVT_HW_INIT_DONE,  /* 等待的事件位 */
                      pdFALSE,           /* 不清除（其他任务也要看这个位） */
                      pdTRUE,            /* 等所有指定位都置1 */
                      portMAX_DELAY);    /* 永久等待 */
  printf("[Acq] Hardware init done, starting acquisition.\r\n");

  /* 获取当前时间作为基准 */
  last_wake_time = xTaskGetTickCount();

  /* 重新打开所有在自检阶段被识别出存在传感器的通道，准备进行周期性采数 */
  bool any_channel_opened = false;
  for (int i = 0; i < 3; i++) {
    if (WQInterface.Channel[i].connected == 1 && WQInterface.Channel[i].type != SENSOR_TYPE_NONE) {
	    SelfExam_OpenChannel(i + 1);   //把所有通道重新上电
      any_channel_opened = true;
      printf("[Acq] Channel %d ON (type=%d)\r\n", i + 1, WQInterface.Channel[i].type);
    }
  }
  if (any_channel_opened) {
    vTaskDelay(pdMS_TO_TICKS(2000)); // 统一延时等待稳定
  }

  while (1) {
    /* 1. 清空快照箱子 (memset 已经把所有 bool 清零为 false 了) */
    memset(&data, 0, sizeof(data));

    /* 2. 获取频率参数（必须在 memset 之后，否则会被冲掉） */
    data.acq_frequency = g_acquisition_frequency;
    data.upload_frequency = Upload_GetFrequency(); // 获取上传频率

    /* ==========================================
     * 🌟 灵魂映射：把 3个物理通道状态 -> 映射成 7个逻辑传感器状态
     * ========================================== */
    /* 轮询三个通道 */
    for (int i = 0; i < 3; i++) {
      // 如果这个通道物理上是开启且连接的
      if (WQInterface.Channel[i].connected == 1) {

                // 根据这个通道绑定的传感器类型，点亮快照里对应的标志位
                switch (WQInterface.Channel[i].type) {
                    case SENSOR_TYPE_COD:
                        data.cod_connected = true;
                        break;
                    case SENSOR_TYPE_CDOM:
                        data.cdom_connected = true;
                        break;
                    case SENSOR_TYPE_CHL:
                        data.chl_connected = true;
                        break;
                    case SENSOR_TYPE_PH:
                        data.ph_connected = true;
                        break;
                    case SENSOR_TYPE_DO:
                        data.do_connected = true;
                        break;
                    case SENSOR_TYPE_SAL:
                        data.sal_connected = true;
                        break;
                    case SENSOR_TYPE_Y4000:
                        data.y4000_connected = true;
                        break;
                    default:
                        break; // 未知类型不处理
                }
            }
        } // 补充 for 循环的闭合大括号

    // printf("[Acquisition] Starting collection cycle...\r\n");

    /* 1. 采集GPS数据 */
    collect_gps_data(&data);

    /* 2. 采集环境数据（温湿度、电量） */
    collect_environment_data(&data);

    /* 3. 采集水质传感器数据 */
    collect_all_water_sensors(&data);

    /* 4. 通过互斥锁更新全局共享数据 */
    if (xSemaphoreTake(g_acquisition_data_mutex, pdMS_TO_TICKS(100)) ==
        pdTRUE) {
      /* 更新全局数据（覆盖旧数据，使用memcpy复制过去，为什么不用strcpy？因为strcpy只能复制字符串，memcpy什么都能复制）
       */
      memcpy(&g_latest_acquisition_data, &data, sizeof(acquisition_data_t));
      xSemaphoreGive(g_acquisition_data_mutex);
      // printf("[Acquisition] Global data updated\r\n");
    } else {
      // printf("[Acquisition] Failed to acquire mutex, data not updated\r\n");
    }

    /* 5. 等待下一个采集周期 */
    delay_ticks = pdMS_TO_TICKS(g_acquisition_frequency * 1000);
    vTaskDelayUntil(&last_wake_time, delay_ticks);
  }
}

/*============================================================================
 *                          GPS数据采集
 *===========================================================================*/

/**
 * @brief  采集GPS数据
 */
static void collect_gps_data(acquisition_data_t *data) {
  gps_data_t gpsdata;

  if (WQInterface.GPS.GetGPSData(&gpsdata) == 1 && gpsdata.is_valid) {
    data->latitude = gpsdata.latitude;
    data->longitude = gpsdata.longitude;
    data->day = gpsdata.time.day;
    data->month = gpsdata.time.month;
    data->year = gpsdata.time.year;
    data->hour = gpsdata.time.hour;
    data->min  = gpsdata.time.min;
    data->sec = gpsdata.time.sec;
    data->gps_valid = true;
  } else {
    data->gps_valid = false;
  }
}

/*============================================================================
 *                          环境数据采集
 *===========================================================================*/

/**
 * @brief  采集环境数据（温湿度、电量）
 */
static void collect_environment_data(acquisition_data_t *data) {
  float temperature = 0.0f;
  float humidity = 0.0f;
  /* 读取AHT20温湿度 */
	if(WQInterface.AHT20.GetHumiTemp(&temperature,&humidity)){
	data->env_humi =humidity;
     data->env_temp =temperature;
	}

     data->battery = WQInterface.Battery.GetBattery();
}

/*============================================================================
 *                          水质传感器采集
 *===========================================================================*/
/**
 * @brief  采集所有已配置的水质传感器
 * @note   GPIO控制已在任务启动和SetChannelState中处理，此处只负责采集
 */
static void collect_all_water_sensors(acquisition_data_t *data) {
    for (int i = 0; i < 3; i++) {
        // 如果通道开关是开的，且这个通道在自检时已经绑定了"读函数"
        if (WQInterface.Channel[i].connected == 1 && WQInterface.Channel[i].Read != NULL) {
            // 直接执行！不需要问你是谁，不需要 switch-case
            WQInterface.Channel[i].Read(data);

            /* ⚠️ RS485帧间间隔：三个传感器共享同一条半双工总线，
             * 查完一个传感器后必须等总线完全静默，才能查下一个。
             * 100ms 足以满足 Modbus RTU 3.5字符帧间隔要求（9600bps下约4ms），
             * 并给传感器充足的时间释放总线。 */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/*============================================================================
 *                          频率控制API
 *===========================================================================*/

/**
 * @brief  设置采集频率
 */
bool Acquisition_SetFrequency(uint16_t freq_seconds) {
  if (freq_seconds < ACQ_FREQ_MIN || freq_seconds > ACQ_FREQ_MAX) {
    printf("[Acquisition] Invalid frequency: %d (valid: %d~%d)\r\n",
           freq_seconds, ACQ_FREQ_MIN, ACQ_FREQ_MAX);
    return false;
  }

  g_acquisition_frequency = freq_seconds;
  printf("[Acquisition] Frequency set to %d seconds\r\n", freq_seconds);
  return true;
}

/**
 * @brief  获取当前采集频率
 */
uint16_t Acquisition_GetFrequency(void) { return g_acquisition_frequency; }

/*============================================================================
 *                          通道控制API
 *===========================================================================*/

/**
 * @brief  设置传感器通道状态
 * @note   状态变化时自动控制GPIO（开启时延时2秒等待继电器稳定）
 */
bool Acquisition_SetChannelState(uint8_t channel, uint8_t state) {
  if (channel < 1 || channel > 3) return false;
  
  uint8_t old_state = WQInterface.Channel[channel - 1].connected;

  /* 状态未变化，直接返回 */
  if (old_state == (state ? 1 : 0)) {
    return true;
  }

  /* 更新状态 */
  WQInterface.Channel[channel - 1].connected = state ? 1 : 0;

  /* 根据状态变化控制GPIO */
  if (state == 1 && old_state == 0) {
    /* 从关闭到开启: 打开继电器并延时稳定 */
    printf("[Acquisition] Channel %d: Opening relay...\r\n", channel);
    SelfExam_OpenChannel(channel);
    vTaskDelay(pdMS_TO_TICKS(2000)); // 等待继电器稳定
    printf("[Acquisition] Channel %d: Relay opened and stabilized\r\n",
           channel);
  } else if (state == 0 && old_state == 1) {
    /* 从开启到关闭: 关闭继电器 */
    printf("[Acquisition] Channel %d: Closing relay...\r\n", channel);
    SelfExam_CloseChannel(channel);
  }

  printf("[Acquisition] Channel %d state changed: %d -> %d\r\n", channel,
         old_state, WQInterface.Channel[channel - 1].connected);
  return true;
}

/**
 * @brief  设置传感器通道类型
 * @note   此函数主要用于手动覆盖自动识别的传感器类型（不推荐使用）
 *         正常情况下，传感器类型由上电自检自动识别，无需手动设置
 
bool Acquisition_SetChannelType(uint8_t channel, sensor_type_t type) {
  switch (channel) {
  case 1:
    channel1_sensor = type;
    break;
  case 2:
    channel2_sensor = type;
    break;
  case 3:
    channel3_sensor = type;
    break;
  default:
    return false;
  }

  printf("[Acquisition] Channel %d type manually set to %d\r\n", channel, type);
  return true;
}
*/

