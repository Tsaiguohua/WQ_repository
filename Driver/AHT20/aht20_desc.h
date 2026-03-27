#ifndef AHT20_DESC_H
#define AHT20_DESC_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// AHT20私有描述符 - 仅供驱动内部使用
// 定义硬件配置参数，使驱动可配置化
//////////////////////////////////////////////////////////////////////////////////

/* 默认设备句柄 */
#define AHT20_DEFAULT (&aht20_default_dev)//宏定义只是文本替换，

/* AHT20设备描述符 - 包含硬件配置 */
struct aht20_desc {
  uint8_t addr_write; // I2C写地址 (7位地址左移1位)
  uint8_t addr_read;  // I2C读地址
  bool calibrated;    // 校准标志
};
        
/* 默认设备实例（大多数情况只用一个AHT20） */
extern struct aht20_desc aht20_default_dev;



#endif 
