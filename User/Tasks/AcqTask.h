#ifndef ACQTASK_H
#define ACQTASK_H


#include "FreeRTOS.h" // 必须在 semphr.h 之前包含
#include "semphr.h"   // FreeRTOS 信号量支持
#include <stdbool.h>
#include <stdint.h>
#include "WQInterface.h"

//////////////////////////////////////////////////////////////////////////////////
// 数据采集任务 - FreeRTOS版本
//
// 功能说明：
//   1. 周期性采集所有传感器数据（GPS、温湿度、水质传感器）
//   2. 支持通过MQTT/串口命令动态设置采集频率（1秒~上千秒）
//   3. 采集完成后将数据发送到队列，供上传任务消费
//
// 传感器类型：
//   - GPS: 经纬度、时间
//   - AHT20: 环境温湿度
//   - Voltage: 电池电量
//   - Y4000: PH/DO/SAL/ATM（多参数）
//   - COD: 水温/COD/TOC/浊度
//   - CDOM: 有色溶解有机物
//   - CHL: 叶绿素
//   - PH/DO/SAL: 独立传感器
//
// 创建日期: 2026/01/03
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          采集频率配置
 *===========================================================================*/

#define ACQ_FREQ_MIN 1     // 最小采集频率（秒）
#define ACQ_FREQ_MAX 3600  // 最大采集频率（秒）= 1小时
#define ACQ_FREQ_DEFAULT 5 // 默认采集频率（秒）

/*============================================================================
 *                          传感器通道配置
 * 与裸机版本保持一致：支持3个独立通道
 *===========================================================================*/

/*============================================================================
 *                          采集数据结构体
 * 包含所有传感器的采集数据
 *===========================================================================*/

typedef struct {
  /* 时间戳 */
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;

  /* GPS数据 */
  float latitude;  // 纬度
  float longitude; // 经度
  bool gps_valid;  // GPS定位有效

  /* 环境数据 */
  float env_temp;  // 环境温度（℃）
  float env_humi;  // 环境湿度（%）
  uint8_t battery; // 电池电量（%）

  /* 水质数据 - COD传感器 */
  float water_temp; // 水温（℃）
  float cod;        // COD值（mg/L）
  float toc;        // TOC值（mg/L）
  float tur;        // 浊度（NTU）

  /* 水质数据 - 其他传感器 */
  float cdom;   // CDOM值
  float chl;    // 叶绿素值
  float ph;     // PH值（单独传感器或Y4000）
  float do_val; // 溶解氧值（单独传感器或Y4000）
  float sal;    // 盐度值（单独传感器或Y4000）
  float atm;    // 大气压（Y4000）
  
  //水质传感器的在线状态
  bool cod_connected;   // COD传感器是否在线
  bool cdom_connected;  // CDOM传感器是否在线
  bool chl_connected;   // 叶绿素传感器是否在线
  bool ph_connected;    // PH传感器是否在线
  bool do_connected;    // 溶解氧传感器是否在线
  bool sal_connected;   // 盐度传感器是否在线
  bool y4000_connected; // Y4000多参数传感器是否在线
  
  /* 通道状态已全部移至中间层 WQInterface.Channel[i].connected 和 .type 管理
   * 上传时请直接访问 WQInterface，不再在此结构体中冗余存储 */

  /* 采集配置 */
  uint16_t acq_frequency;    // 当前采集频率（秒）
  uint16_t upload_frequency; // 当前上传频率（秒）

} acquisition_data_t;

/*============================================================================
 *                          全局变量声明
 *===========================================================================*/

/* 当前采集频率（秒） */
extern uint16_t g_acquisition_frequency;
 
/* 最新采集数据缓冲（所有任务共享）*/
extern acquisition_data_t g_latest_acquisition_data;

/* 获取数据的互斥锁 */
extern SemaphoreHandle_t g_acquisition_data_mutex;

/* 传感器类型由自检模块自动识别（定义在 self_exam.h）
 * 这些变量由上电自检时自动填充，无需手动配置
 * 引用时请包含 self_exam.h 头文件
 * - extern sensor_type_t channel1_sensor;
 * - extern sensor_type_t channel2_sensor;
 * - extern sensor_type_t channel3_sensor;
 */

/*============================================================================
 *                          API函数声明
 *===========================================================================*/

/**
 * @brief  初始化采集模块
 * @note   在FreeRTOS任务启动前调用
 */
void Acquisition_Init(void);

/**
 * @brief  采集任务函数
 * @param  pvParameters: FreeRTOS任务参数（未使用）
 * @note   由xTaskCreate创建
 */
void Acquisition_Task(void *pvParameters);

/**
 * @brief  设置采集频率
 * @param  freq_seconds: 采集间隔（秒），范围1~3600
 * @return true:设置成功, false:参数无效
 */
bool Acquisition_SetFrequency(uint16_t freq_seconds);

/**
 * @brief  获取当前采集频率
 * @return 当前采集间隔（秒）
 */
uint16_t Acquisition_GetFrequency(void);

/**
 * @brief  设置传感器通道状态
 * @param  channel: 通道号（1/2/3）
 * @param  state: 0=关闭, 1=开启
 * @return true:设置成功
 */
bool Acquisition_SetChannelState(uint8_t channel, uint8_t state);

/**
 * @brief  设置传感器通道类型
 * @param  channel: 通道号（1/2/3）
 * @param  type: 传感器类型
 * @return true:设置成功
 */
bool Acquisition_SetChannelType(uint8_t channel, sensor_type_t type);






#endif

