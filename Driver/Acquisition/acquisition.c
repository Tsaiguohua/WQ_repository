#include "acquisition.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* 传感器驱动头文件 */
#include "aht20.h"
#include "cdom.h"
#include "chl.h"
#include "cod.h"
#include "do.h"
#include "gps.h"
#include "ph.h"
#include "sal.h"
#include "voltage.h"
#include "y4000.h"

/* 自检模块（获取自动识别的传感器类型,并控制通道GPIO） */
#include "self_exam.h"

/* 上传模块（获取上传频率） */
#include "upload.h"

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

/* 传感器通道状态 */
uint8_t onechannel_state = 0;
uint8_t twochannel_state = 0;
uint8_t threechannel_state = 0;

/* 引用自检模块的传感器类型变量（自动识别，无需手动配置）
 * 这些变量在上电自检时自动填充，采集任务直接使用
 */
extern sensor_type_t channel1_sensor; // 通道1自动识别的传感器类型
extern sensor_type_t channel2_sensor; // 通道2自动识别的传感器类型
extern sensor_type_t channel3_sensor; // 通道3自动识别的传感器类型

/* 最新采集数据（全局共享数据） */
acquisition_data_t g_latest_acquisition_data;

/* 保护采集数据的互斥锁 */
SemaphoreHandle_t g_acquisition_data_mutex = NULL;

/*============================================================================
 *                          内部函数声明
 *===========================================================================*/

static void collect_gps_data(acquisition_data_t *data);
static void collect_environment_data(acquisition_data_t *data);
static void collect_water_sensor_by_type(sensor_type_t type,
                                         acquisition_data_t *data);
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

  /* 获取当前时间作为基准 */
  last_wake_time = xTaskGetTickCount();

  // printf("[Acquisition] Task started\r\n");

  /* 启动时根据Flash恢复的状态开启通道（类似裸机VoiceControl） */
  if (onechannel_state == 1) {
    // printf("[Acquisition] Channel 1 initial power on...\r\n");
    SelfExam_OpenChannel(1);
    vTaskDelay(pdMS_TO_TICKS(2000)); // 等待继电器稳定，只延时一次
  }
  if (twochannel_state == 1) {
    // printf("[Acquisition] Channel 2 initial power on...\r\n");
    SelfExam_OpenChannel(2);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  if (threechannel_state == 1) {
    // printf("[Acquisition] Channel 3 initial power on...\r\n");
    SelfExam_OpenChannel(3);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  while (1) {
    /* 清空数据结构 */
    memset(&data, 0, sizeof(data));
    data.acq_frequency = g_acquisition_frequency;
    data.upload_frequency = Upload_GetFrequency(); // 获取上传频率
    data.channel1_state = onechannel_state;
    data.channel2_state = twochannel_state;
    data.channel3_state = threechannel_state;
    data.channel1_sensor = channel1_sensor; // 自动识别
    data.channel2_sensor = channel2_sensor; // 自动识别
    data.channel3_sensor = channel3_sensor; // 自动识别

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

    /* 打印采集结果摘要 */
    // printf("[Acquisition] Cycle complete: GPS=%s, Temp=%.1fC, Humi=%.1f%%, "
    //        "Batt=%d%%\r\n",
    //        data.gps_valid ? "OK" : "NG", data.env_temp, data.env_humi,
    //        data.battery);

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
  gps_data_t gps;

  /* 解析GPS缓冲区 */
  GPS_Parse();

  /* 获取GPS数据 */
  if (GPS_GetData(&gps) && gps.is_valid) {
    data->latitude = gps.latitude;
    data->longitude = gps.longitude;
    data->year = gps.time.year;
    data->month = gps.time.month;
    data->day = gps.time.day;
    data->hour = gps.time.hour;
    data->min = gps.time.min;
    data->sec = gps.time.sec;
    data->gps_valid = true;

    // printf("[GPS] Lat=%.6f, Lon=%.6f, Time=%04d-%02d-%02d
    // %02d:%02d:%02d\r\n",
    //        data->latitude, data->longitude, data->year, data->month,
    //        data->day, data->hour, data->min, data->sec);
  } else {
    data->gps_valid = false;
    // printf("[GPS] No valid fix\r\n");
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
  if (aht20_start_measurement(AHT20_t *dev)) {
    vTaskDelay(pdMS_TO_TICKS(80)); // 等待测量完成

    if (aht20_read_measurement(AHT20_t *dev, &temperature, &humidity)) {
      data->env_temp = temperature;
      data->env_humi = humidity;
      // printf("[AHT20] Temp=%.1fC, Humi=%.1f%%\r\n", temperature, humidity);
    } else {
      // printf("[AHT20] Read failed\r\n");
    }
  } else {
    // printf("[AHT20] Start measurement failed\r\n");
  }

  /* 读取电池电量 */
  extern uint8_t battery; // 来自Voltage模块
  data->battery = battery;
  // printf("[Voltage] Battery=%d%%\r\n", data->battery);
}

/*============================================================================
 *                          水质传感器采集
 *===========================================================================*/

/**
 * @brief  根据传感器类型采集水质数据
 */
static void collect_water_sensor_by_type(sensor_type_t type,
                                         acquisition_data_t *data) {
  switch (type) {
  case SENSOR_TYPE_COD: {
    /* COD传感器：读取水温、COD、TOC、浊度 */
    cod_data_t cod_result;
    if (cod_read(&cod_result) && cod_result.valid) {
      data->water_temp = cod_result.temp;
      data->cod = cod_result.cod;
      data->toc = cod_result.toc;
      data->tur = cod_result.tur;
      printf("[COD] Temp=%.1f, COD=%.2f, TOC=%.2f, TUR=%.2f\r\n",
             cod_result.temp, cod_result.cod, cod_result.toc, cod_result.tur);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_CDOM: {
    /* CDOM传感器 */
    float cdom_val;
    if (cdom_read(&cdom_val)) {
      data->cdom = cdom_val;
      printf("[CDOM] Value=%.2f\r\n", cdom_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_CHL: {
    /* 叶绿素传感器 */
    float chl_val;
    if (chl_read(&chl_val)) {
      data->chl = chl_val;
      printf("[CHL] Value=%.2f\r\n", chl_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_PH: {
    /* 独立PH传感器 */
    float ph_val;
    if (ph_read(&ph_val)) {
      data->ph = ph_val;
      printf("[PH] Value=%.2f\r\n", ph_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_DO: {
    /* 独立DO传感器 */
    float do_val;
    if (do_read(&do_val)) {
      data->do_val = do_val;
      printf("[DO] Value=%.2f\r\n", do_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_SAL: {
    /* 独立SAL传感器 */
    float sal_val;
    if (sal_read(&sal_val)) {
      data->sal = sal_val;
      printf("[SAL] Value=%.2f\r\n", sal_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_Y4000: {
    /* Y4000多参数母体：一次读取PH/DO/SAL */
    y4000_data_t y4000_result;
    if (y4000_read(&y4000_result) && y4000_result.valid) {
      data->ph = y4000_result.ph_value;
      data->do_val = y4000_result.do_value;
      data->sal = y4000_result.sal_value;
      printf("[Y4000] PH=%.2f, DO=%.2f, SAL=%.2f\r\n", y4000_result.ph_value,
             y4000_result.do_value, y4000_result.sal_value);
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 再读取气压 */
    float atm_val;
    if (y4000_read_atm(&atm_val)) {
      data->atm = atm_val;
      printf("[Y4000] ATM=%.2f\r\n", atm_val);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    break;
  }

  case SENSOR_TYPE_NONE:
  default:
    break;
  }
}

/**
 * @brief  采集所有已配置的水质传感器
 * @note   GPIO控制已在任务启动和SetChannelState中处理，此处只负责采集
 */
static void collect_all_water_sensors(acquisition_data_t *data) {
  // printf("[Acquisition] Collecting water sensors...\r\n");

  /* 通道1: 如果状态开启则采集 */
  if (onechannel_state == 1) {
    // printf("[Acquisition] Channel 1 collecting (type=%d)...\r\n",
    //        channel1_sensor);
    collect_water_sensor_by_type(channel1_sensor, data);
  }

  /* 通道2: 如果状态开启则采集 */
  if (twochannel_state == 1) {
    // printf("[Acquisition] Channel 2 collecting (type=%d)...\r\n",
    //        channel2_sensor);
    collect_water_sensor_by_type(channel2_sensor, data);
  }

  /* 通道3: 如果状态开启则采集 */
  if (threechannel_state == 1) {
    // printf("[Acquisition] Channel 3 collecting (type=%d)...\r\n",
    //        channel3_sensor);
    collect_water_sensor_by_type(channel3_sensor, data);
  }

  // printf("[Acquisition] Water sensors done\r\n");
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
  uint8_t *channel_state_ptr = NULL;
  uint8_t old_state = 0;

  /* 获取对应通道的状态变量指针 */
  switch (channel) {
  case 1:
    channel_state_ptr = &onechannel_state;
    old_state = onechannel_state;
    break;
  case 2:
    channel_state_ptr = &twochannel_state;
    old_state = twochannel_state;
    break;
  case 3:
    channel_state_ptr = &threechannel_state;
    old_state = threechannel_state;
    break;
  default:
    return false;
  }

  /* 状态未变化，直接返回 */
  if (old_state == (state ? 1 : 0)) {
    return true;
  }

  /* 更新状态 */
  *channel_state_ptr = state ? 1 : 0;

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
         old_state, *channel_state_ptr);
  return true;
}

/**
 * @brief  设置传感器通道类型
 * @note   此函数主要用于手动覆盖自动识别的传感器类型（不推荐使用）
 *         正常情况下，传感器类型由上电自检自动识别，无需手动设置
 */
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
