#ifndef __IIC_H
#define __IIC_H
#include "sys.h"


typedef struct
{
	GPIO_TypeDef * SDA_PORT;
	GPIO_TypeDef * SCL_PORT;
	uint16_t SDA_PIN;
	uint16_t SCL_PIN;
	uint32_t RCC_AHB1Periph;
}iic_bus_t;

// IIC所有操作函数
void IIC_Init(iic_bus_t *bus);                 // 初始化IIC的IO口
void IIC_Start(iic_bus_t *bus);                // 发送IIC开始信号
void IIC_Stop(iic_bus_t *bus);                 // 发送IIC停止信号
void IIC_Send_Byte(iic_bus_t *bus,unsigned char cSendByte);          // IIC发送一个字节
uint8_t IIC_Receive_Byte(iic_bus_t *bus); // IIC读取一个字节
uint8_t IIC_Wait_Ack(iic_bus_t *bus);               // IIC等待ACK信号
void IICSend_Ack(iic_bus_t *bus);                  // IIC发送ACK信号
void IICSend_NAck(iic_bus_t *bus);                 // IIC不发送ACK信号

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t data);
uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t length,uint8_t buff[]);
unsigned char IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg);
uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[]);
#endif
