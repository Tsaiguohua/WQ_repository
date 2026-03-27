#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

/* UTC时间结构体 */
typedef struct {
  uint16_t year; // 年（2000-2099）
  uint8_t month; // 月（1-12）
  uint8_t day;   // 日（1-31）
  uint8_t hour;  // 时（0-23）
  uint8_t min;   // 分（0-59）
  uint8_t sec;   // 秒（0-59）
} gps_time_t;

/* GPS数据结构体（纯数据记录） */
typedef struct {
  float latitude;  // 纬度（度，正为北纬，负为南纬）
  float longitude; // 经度（度，正为东经，负为西经）
  gps_time_t time; // 北京时间
  bool is_valid;   // 定位是否有效
} gps_data_t;

/*============================================================================
 *                          API函数
 *===========================================================================*/

/**
 * @brief  初始化GPS硬件（USART3并配置中断）
 * @param  rx_callback 当接收到完整的NMEA帧（$GNRMC）时的回调函数，该函数在中断中执行
 */
void GPS_Init(void (*rx_callback)(void));

/**
 * @brief  获取缓存内完整的原始NMEA帧
 * @param  buf       调用方提供的缓冲
 * @param  max_len   缓冲的最大长度
 * @return true成功获取, false无新数据
 */
bool GPS_GetRawFrame(char *buf, uint16_t max_len);

/**
 * @brief  解析NMEA帧
 * @param  frame_buf 原始NMEA字符串数据
 * @param  parsed_data 输出的解析结构体
 * @return true解析成功, false解析失败或格式不对
 */
bool GPS_ParseNMEA(const char *frame_buf, gps_data_t *parsed_data);

#endif
