#include "aht20.h"
#include "FreeRTOS.h"
#include "aht20_desc.h"
#include "iic.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// AHT20温湿度传感器驱动 - 面向对象风格
// 采用描述符+句柄模式 + 异步测量拆分
// aht20.h是门面，对外展示的菜单（只有函数名）
// aht20_desc.h是配置，设备的身份证（iic地址等参数）
// aht20.c是核心，真正干活的
// 创建日期: 2025/12/30
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////
							    
iic_bus_t AHT_bus = 
{
	.SDA_PORT = GPIOF,
	.SCL_PORT = GPIOF,
	.SDA_PIN  = GPIO_Pin_2,
	.SCL_PIN  = GPIO_Pin_3,
	.RCC_AHB1Periph = RCC_AHB1Periph_GPIOF,
};

AHT20_t dev = {
	.bus = &AHT_bus,
	.addr_write = 0x70,
	.addr_read = 0x71,
	.calibrated = false,
};
	

/*============================================================================
 *                          私有函数 - IIC通信
 * 注意：IIC底层移植的正点原子基于freertos修改后的delay_us()实现精确时序
 *===========================================================================*/

/**
 * @brief  向AHT20发送命令
 */
static bool aht20_write_cmd(AHT20_t *dev, uint8_t cmd, uint8_t *data,
                            uint8_t len) {
  uint8_t i;

  IIC_Start(dev->bus);                           
  IIC_Send_Byte(dev->bus, dev->addr_write);
  if (IIC_Wait_Ack(dev->bus))
    return false;

  IIC_Send_Byte(dev->bus,cmd);
  if (IIC_Wait_Ack(dev->bus))
    return false;

  for (i = 0; i < len; i++) {
    IIC_Send_Byte(dev->bus,data[i]);
    if (IIC_Wait_Ack(dev->bus))
      return false;
  }
  IIC_Stop(dev->bus);

  return true;
}

/**
 * @brief  读取AHT20状态字节
 */
static bool aht20_read_status(AHT20_t *dev, uint8_t *status) {
  IIC_Start(dev->bus);
  IIC_Send_Byte(dev->bus,dev->addr_read);
  if (IIC_Wait_Ack(dev->bus))
    return false;

  *status = IIC_Receive_Byte(dev->bus); 
  IICSend_NAck(dev->bus);
  IIC_Stop(dev->bus);

  return true;
}

/**
 * @brief  检查AHT20是否忙
 */
static bool aht20_is_busy(AHT20_t *dev) {
  uint8_t status;
  if (!aht20_read_status(dev,&status)) // 如果读取状态失败，就是readstatus返回false的话，说明通信有问题，则返回真（busy）
    return true;
  return (status & 0x80) != 0; // 说明iic通信成功，位运算，返回true或者flase
}

/**
 * @brief  检查AHT20是否已校准
 */
static bool aht20_is_calibrated(AHT20_t *dev) {
  uint8_t status;
  if (!aht20_read_status(dev, &status))
    return false;
  return (status & 0x08) != 0;
}

/*============================================================================
 *                          公开API
 * 注意：任务级延时使用vTaskDelay()让出CPU
 *===========================================================================*/

/**
 * @brief  初始化AHT20
 */

bool aht20_init(AHT20_t *dev) {
  uint8_t init_data[2] = {0x08, 0x00};

  IIC_Init(dev->bus);
  vTaskDelay(pdMS_TO_TICKS(40)); // 上电等待40ms

  // 检查是否已校准
  if (aht20_is_calibrated(dev)) {
    dev->calibrated = true;
    return true;
  }

  // 发送初始化命令 0xBE
  if (!aht20_write_cmd(dev, 0xBE, init_data, 2))
    return false;

  vTaskDelay(pdMS_TO_TICKS(10));

  // 再次检查校准状态
  dev->calibrated = aht20_is_calibrated(dev);
  return dev->calibrated;
}

/**
 * @brief  启动测量（异步）
 */
bool aht20_start_measurement(AHT20_t *dev) {
  uint8_t measure_data[2] = {0x33, 0x00};
  return aht20_write_cmd(dev, 0xAC, measure_data, 2);
}

/**
 * @brief  等待测量完成
 */
bool aht20_wait_for_measurement(AHT20_t *dev) {
  // 典型测量时间约75ms，最大等待200ms
  for (uint32_t t = 0; t < 20; t++) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!aht20_is_busy(dev)) {
      return true;
    }
  }
  return false; // 超时
}

/**
 * @brief  读取测量结果
 */
bool aht20_read_measurement(AHT20_t *dev, float *temperature, float *humidity) {
  uint8_t buf[6];
  uint8_t i;
  uint32_t humi_raw, temp_raw;

  // 读取6字节数据
  IIC_Start(dev->bus);
  IIC_Send_Byte(dev->bus,dev->addr_read);
  if (IIC_Wait_Ack(dev->bus))
    return false;

  for (i = 0; i < 5; i++) {
    buf[i] = IIC_Receive_Byte(dev->bus); 
    IICSend_Ack(dev->bus); // Send ACK
  }
  buf[5] = IIC_Receive_Byte(dev->bus); 
  IICSend_NAck(dev->bus); // Send NACK
  IIC_Stop(dev->bus);

  // 数据解析
  humi_raw = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) |((buf[3] & 0xF0) >> 4);

  temp_raw =
      ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

  // 转换为物理量
  *humidity = (float)humi_raw * 100.0f / 1048576.0f;
  *temperature = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;

  return true;
}

/**
 * @brief  一步到位读取温湿度（封装异步三步法）
 * @note   内部会：启动测量 -> 等待完成 -> 读取结果
 */
bool aht20_read_temp_humi(AHT20_t *dev, float *temperature, float *humidity) {
  // 步骤1: 启动测量
  if (!aht20_start_measurement(dev)) {
    return false;
  }

  // 步骤2: 等待测量完成（内部使用vTaskDelay让出CPU）
  if (!aht20_wait_for_measurement(dev)) {
    return false;
  }

  // 步骤3: 读取结果
  return aht20_read_measurement(dev, temperature, humidity);
}
