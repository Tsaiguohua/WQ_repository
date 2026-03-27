#ifndef AHT20_H
#define AHT20_H

#include <stdbool.h>
#include <stdint.h>
#include "iic.h"
//////////////////////////////////////////////////////////////////////////////////
// AHT20温湿度传感器驱动 - 面向对象风格
//
//
//
// 采用描述符+句柄模式，支持多实例
// 创建日期: 2025/12/30
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

typedef struct 
{
	iic_bus_t *bus;
	uint8_t addr_write ;
	uint8_t addr_read;
	bool calibrated;
} AHT20_t;

/**
 * @brief  初始化AHT20传感器
 * @param  dev: AHT20设备描述符
 * @return true:成功, false:失败
 */
bool aht20_init(AHT20_t *dev);

/**
 * @brief  启动AHT20测量（异步，不等待结果）
 * @param  dev: AHT20设备描述符
 * @return true:成功, false:失败
 */
bool aht20_start_measurement(AHT20_t *dev);

/**
 * @brief  等待AHT20测量完成（使用vTaskDelay让出CPU）
 * @param  dev: AHT20设备描述符
 * @return true:测量完成, false:超时
 */
bool aht20_wait_for_measurement(AHT20_t *dev);

/**
 * @brief  读取AHT20测量结果
 * @param  dev: AHT20设备描述符
 * @param  temperature: 输出温度值(℃)
 * @param  humidity: 输出湿度值(%)
 * @return true:成功, false:失败
 */
bool aht20_read_measurement(AHT20_t *dev, float *temperature, float *humidity);

/**
 * @brief  一步到位读取温湿度（封装异步三步法）
 * @param  dev: AHT20设备描述符
 * @param  temperature: 输出温度值(℃)
 * @param  humidity: 输出湿度值(%)
 * @return true:成功, false:失败
 * @note   内部会自动执行：启动测量 -> 等待完成 -> 读取结果
 */
bool aht20_read_temp_humi(AHT20_t *dev, float *temperature, float *humidity);

#endif
