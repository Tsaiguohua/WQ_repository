#ifndef WQINTERFACE_H
#define WQINTERFACE_H

#include "gps.h"
#include <stdbool.h>


// ===== 硬件裁剪开关，0是不用，1是使用=====
#define WQ_USE_HARDWARE  1
#if WQ_USE_HARDWARE
  #define WQ_USE_SENSOR_COD    1
  #define WQ_USE_SENSOR_CDOM   1
  #define WQ_USE_SENSOR_CHL    1
  #define WQ_USE_SENSOR_PH     1
  #define WQ_USE_SENSOR_DO     1
  #define WQ_USE_SENSOR_SAL    1
  #define WQ_USE_SENSOR_Y4000  1
  #define WQ_USE_AHT20         1
  #define WQ_USE_Battery       1
  #define WQ_USE_GPS           1
  #define WQ_USE_TF            1
  #define WQ_USE_Command       1
  #define WQ_USE_POWER_SWITCH  1    
  #define WQ_USE_NetWork       1
  #define WQ_USE_Display       1
 #endif
 
 
// ===== 传感器类型枚举 =====
typedef enum {
  SENSOR_TYPE_NONE = 0x00,  // 未配置
  SENSOR_TYPE_COD  = 0x01,  // COD传感器
  SENSOR_TYPE_CDOM = 0x02,  // CDOM传感器
  SENSOR_TYPE_CHL  = 0x03,  // 叶绿素传感器
  SENSOR_TYPE_PH   = 0x04,  // PH传感器
  SENSOR_TYPE_DO   = 0x05,  // DO传感器
  SENSOR_TYPE_SAL  = 0x06,  // SAL传感器
  SENSOR_TYPE_Y4000 = 0x07, // Y4000多参数母体
} sensor_type_t;

// ===== 水质传感器接口=====
typedef struct {
    uint8_t  connected;        // 是否连接
    uint8_t  channel;          // 通道号 1/2/3
    uint8_t  type;             // sensor_type_t
    uint8_t  (*Init)(void);
    uint8_t  (*Read)(void *out_data);  // 读数据，输出到结构体
} WQ_Channel_InterfaceTypeDef;

//===== GPS接口 =====
typedef struct {
    uint8_t  (*Init)(void (*rx_callback)(void));
    bool     (*GetRawFrame)(char *buf, uint16_t max_len);
    bool     (*ParseNMEA)(const char *frame_buf, gps_data_t *parsed_data);
    uint8_t  (*GetGPSData)(gps_data_t *data);
} WQ_GPS_InterfaceTypeDef;

// ===== AHT20传感器接口 =====
typedef struct {
    uint8_t  (*Init)(void);
    uint8_t  (*GetHumiTemp)(float *temp, float *humi);
} WQ_AHT20_InterfaceTypeDef;


// ===== 电量检测接口 =====
typedef struct {
    uint8_t  (*Init)(void);
    uint8_t  (*GetBattery)(void);
} WQ_Battery_InterfaceTypeDef;

// ===== 电源开关接口 =====
typedef struct {
    void (*Init)(void);
    void (*SensorsON)(void);
    void (*SensorsOFF)(void);
    void (*NetworkON)(void);
    void (*NetworkOFF)(void);
} WQ_Power_InterfaceTypeDef;

// ===== 网络接口（4G MQTT）=====
typedef struct {
    uint8_t  connected;
    uint8_t  (*Init)(void);
    uint8_t  (*Send)(const char *str);  // 发送 JSON
} WQ_Network_InterfaceTypeDef;

// ===== 本地存储接口（TF卡）=====
typedef struct {
    uint8_t (*Init)(void);
    uint8_t (*WriteCSVHeader)(const char *header_str);
    uint8_t (*WriteCSV)(const char *csv_str);
    uint8_t (*WriteTXT)(const char *txt_str);
} WQ_Storage_InterfaceTypeDef;

// ===== 命令接口 =====
typedef struct {
    void (*Restart)(void);
    void (*SetBuzzer)(bool on);
    void (*ScanAndBindChannels)(void);
    void (*Set4GPower)(bool on);  // 4G模块电源控制
} WQ_System_InterfaceTypeDef;

// ===== OLED显示接口 =====
typedef struct {
    uint8_t (*Init)(void);
    uint8_t (*Clear)(void);
    uint8_t (*ShowBootAnimation)(void);
    uint8_t (*ShowMainScreen)(const char *year, const char *time, const char *battery, const char *temp, const char *freq, const char *hum, uint8_t ch1, uint8_t ch2, uint8_t ch3);
} WQ_Display_InterfaceTypeDef;

// ===== 总接口（全局单例）=====
typedef struct {
    WQ_Channel_InterfaceTypeDef  Channel[3];
    WQ_Power_InterfaceTypeDef    Power;
    WQ_GPS_InterfaceTypeDef      GPS;
    WQ_Battery_InterfaceTypeDef  Battery;
    WQ_Network_InterfaceTypeDef  Network;
    WQ_AHT20_InterfaceTypeDef    AHT20;
    WQ_System_InterfaceTypeDef   System;
    WQ_Display_InterfaceTypeDef  Display;
    WQ_Storage_InterfaceTypeDef  Storage;
} WQ_InterfaceTypeDef;


extern WQ_InterfaceTypeDef WQInterface;
    
void WQ_Interface_Bind(void);  

#endif


