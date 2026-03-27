#include "iic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//IIC代码	  延时是移植的正点原子基于freertos修改后的delay_us()实现精确时序 
//软件模拟iic，其中PF2是SCL，PF3是SDA,关于为什么模拟iic这里用推挽输出而不是开漏，因为在.h文件里面控制了方向
//发送数据时，把SDA引脚配置为推挽，直接驱动高/低点平，读取时，调用SDA_IN,把SDA引脚配置为上拉模式
//作者：蔡国华
//创建日期:2025/12/23
//版本：V1.0								  
////////////////////////////////////////////////////////////////////////////////// 	 
//////////////////////////////////////////////////////////////////////////////////
// 软件模拟I2C总线驱动 - FreeRTOS版本
//
// 功能说明：
//   本模块使用GPIO模拟I2C通信协议，为AHT20温湿度传感器、OLED显示器等
//   I2C设备提供底层通信支持。采用位banging方式实现标准I2C时序。
//
// 硬件配置：
//   【GPIO引脚】
//   - SCL（时钟）：PF2
//   - SDA（数据）：PF3
//   - GPIO模式：推挽输出（通过宏动态切换输入/输出）
//   - GPIO速度：100MHz
//   - 上拉电阻：内部上拉（GPIO_PuPd_UP）
//
//   【为什么使用推挽而不是开漏？】
//   - 通过SDA_OUT()和SDA_IN()宏动态切换GPIO模式
//   - 发送数据时：推挽输出模式，直接驱动高/低电平
//   - 接收数据时：上拉输入模式，读取外部设备的电平
//   - 这种方式比开漏+外部上拉电阻更灵活
//
// I2C协议时序：
//   【标准I2C时序要求】
//   - SCL时钟频率：约100kHz（标准模式）
//   - 起始条件：SCL为高时，SDA从高到低
//   - 停止条件：SCL为高时，SDA从低到高
//   - 数据传输：SCL为低时改变SDA，SCL为高时读取SDA
//   - ACK应答：从机拉低SDA表示应答（ACK），释放SDA表示非应答（NAK）
//
//   【时序参数】（使用delay_us实现）
//   - 起始/停止条件：4μs延时
//   - 数据传输：2μs延时
//   - ACK等待超时：250次循环
//
// I2C通信流程：
//   【主机发送数据】
//   1. IIC_Start()：发送起始条件
//   2. IIC_Send_Byte(设备地址)：发送从机地址+写标志
//   3. IIC_Wait_Ack()：等待从机应答
//   4. IIC_Send_Byte(数据)：发送数据字节
//   5. IIC_Wait_Ack()：等待从机应答
//   6. IIC_Stop()：发送停止条件
//
//   【主机接收数据】
//   1. IIC_Start()：发送起始条件
//   2. IIC_Send_Byte(设备地址)：发送从机地址+读标志
//   3. IIC_Wait_Ack()：等待从机应答
//   4. data = IIC_Read_Byte(1)：读数据并发送ACK（继续读）
//   5. data = IIC_Read_Byte(0)：读最后一字节并发送NAK（结束）
//   6. IIC_Stop()：发送停止条件
//
// API接口：
//   - IIC_Init()：初始化GPIO引脚
//   - IIC_Start()：产生I2C起始信号
//   - IIC_Stop()：产生I2C停止信号
//   - IIC_Send_Byte(data)：发送一个字节
//   - IIC_Read_Byte(ack)：接收一个字节（ack=1发送ACK，ack=0发送NAK）
//   - IIC_Wait_Ack()：等待从机应答（返回0=成功，1=失败）
//   - IIC_Ack()：主机发送ACK应答
//   - IIC_NAck()：主机发送NAK应答
//
// 连接的I2C设备：
//   - AHT20温湿度传感器（地址0x38）
//   - OLED显示器（地址0x3C）
//   - 其他I2C传感器（根据项目需求）
//
// FreeRTOS兼容性：
//   - delay_us()：使用FreeRTOS修改后的延时函数（基于正点原子）
//   - 软件模拟I2C不使用中断，不需要FreeRTOS同步机制
//   - 可在任意任务中调用（无需互斥锁，因为操作很快）
//   - 如果多任务同时访问同一I2C设备，需要上层添加互斥锁保护
//
// 注意事项：
//   ⚠️ SCL和SDA必须外接上拉电阻（典型值4.7kΩ或10kΩ）
//   ⚠️ delay_us精度影响I2C时序，必须使用精确延时（FreeRTOS版本）
//   ⚠️ 软件模拟I2C速度较慢，不适合高速传输（建议<100kHz）
//   ⚠️ 如果多个任务访问同一设备，需要添加互斥锁保护
//   ⚠️ I2C总线冲突时会导致通信失败，Wait_Ack会返回1
//   ⚠️ 长线传输需要更低的SCL频率（增加delay_us时间）
//
// 调试技巧：
//   - 如果Wait_Ack超时：
//     1. 检查从机地址是否正确（左移1位后加读/写标志）
//     2. 检查上拉电阻是否存在（万用表测SCL/SDA空闲时应为高电平）
//     3. 检查从机是否上电（有些传感器需要初始化延时）
//     4. 使用逻辑分析仪查看波形
//   - 如果数据错误：
//     1. 降低SCL频率（增加delay_us时间）
//     2. 检查线缆长度和质量（建议<30cm）
//     3. 检查供电电压是否稳定
//
// 优缺点对比：
//   【软件模拟I2C优点】
//   - 任意GPIO都可以使用，灵活性高
//   - 不占用硬件I2C资源
//   - 兼容性好，适合各种异常设备
//   
//   【软件模拟I2C缺点】
//   - 速度较慢（受CPU主频和delay精度限制）
//   - 占用CPU时间（无法在传输时处理其他任务）
//   - 时序精度依赖delay函数准确性
//
// 作者：蔡国华
// 创建日期：2025/12/23
// 最后更新：2026/01/14
// 版本：v1.0（基于正点原子代码移植，使用FreeRTOS延时函数）
//////////////////////////////////////////////////////////////////////////////////


static uint8_t SDA_Input(iic_bus_t *bus) {
    if (GPIO_ReadInputDataBit(bus->SDA_PORT, bus->SDA_PIN) == SET) {
        return 1;
    } else {
        return 0;
    }
}

static void SDA_Output(iic_bus_t *bus, uint8_t level) {
    if(level) GPIO_SetBits(bus->SDA_PORT, bus->SDA_PIN);
    else      GPIO_ResetBits(bus->SDA_PORT, bus->SDA_PIN);
}

static void SCL_Output(iic_bus_t *bus, uint8_t level) {
    if(level) GPIO_SetBits(bus->SCL_PORT, bus->SCL_PIN);
    else      GPIO_ResetBits(bus->SCL_PORT, bus->SCL_PIN);
}


// 初始化IIC
void IIC_Init(iic_bus_t *bus)
{			
    GPIO_InitTypeDef  GPIO_InitStructure;

    // 使能总线时钟
    RCC_AHB1PeriphClockCmd(bus->RCC_AHB1Periph, ENABLE);

    // GPIO 统一初始化为 开漏输出 (Open-Drain) + 上拉 (Pull-Up)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // 核心：必须是开漏
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉

    // 初始化 SCL
    GPIO_InitStructure.GPIO_Pin = bus->SCL_PIN;
    GPIO_Init(bus->SCL_PORT, &GPIO_InitStructure);     // 修复了之前的错别字
	
    // 初始化 SDA
    GPIO_InitStructure.GPIO_Pin = bus->SDA_PIN;
    GPIO_Init(bus->SDA_PORT, &GPIO_InitStructure);
	
    // 默认释放总线（高电平）
    SCL_Output(bus, 1);
    SDA_Output(bus, 1);
}

//产生IIC起始信号
void IIC_Start(iic_bus_t *bus)
{
	SDA_Output(bus,1);
	SCL_Output(bus, 1);
     delay_us(2);
     SDA_Output(bus, 0); // SCL高电平时，SDA产生下降沿
     delay_us(1);
     SCL_Output(bus, 0); // 钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(iic_bus_t *bus)
{
	SCL_Output(bus, 0);
	SDA_Output(bus, 0); // 准备产生上升沿
     delay_us(2);
     SCL_Output(bus, 1);
	delay_us(1);
	SDA_Output(bus, 1);	// SCL高电平时，SDA产生上升沿			   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(iic_bus_t *bus)
{
	uint8_t ucErrTime=0;
	SDA_Output(bus,1); // 释放SDA线，等待从机拉低
	delay_us(1);	   
	SCL_Output(bus,1);
	delay_us(1);	 
	while(SDA_Input(bus)) // 读取SDA电平
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(bus);
			return 1;
		}
	}
	SCL_Output(bus,0);//时钟输出0 	   
	return 0;    
} 
//产生ACK应答
void IICSend_Ack(iic_bus_t *bus)
{
	SCL_Output(bus,0);
	SDA_Output(bus,0);// 主机拉低SDA
	delay_us(2);
	SCL_Output(bus,1);
	delay_us(2);
	SCL_Output(bus,0);
}
//不产生ACK应答		    
void IICSend_NAck(iic_bus_t *bus)
{
	SCL_Output(bus,0);
	SDA_Output(bus,1); // 主机释放SDA (高电平代表NACK)
	delay_us(2);
	SCL_Output(bus,1);
	delay_us(2);
	SCL_Output(bus,0);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(iic_bus_t *bus,unsigned char cSendByte)
{                               
    SCL_Output(bus,0);//拉低时钟开始数据传输
    for(int t=0;t<8;t++)
    {              
        SDA_Output(bus, (cSendByte & 0x80) >> 7);
         cSendByte <<= 1; // 左移更规范	    
		delay_us(2);   
	    SCL_Output(bus,1);
		delay_us(2); 
		SCL_Output(bus,0);	
		delay_us(2);
    }	 
} 	    
 // IIC读取一个字节
uint8_t IIC_Receive_Byte(iic_bus_t *bus)
{
    uint8_t  i,receive=0;
    SDA_Output(bus, 1); // 主机释放SDA，把控制权交给从机
    for(i=0;i<8;i++ )
	{
        SCL_Output(bus,0); 
        delay_us(2);
	   SCL_Output(bus,1);
        receive<<=1;
        if(SDA_Input(bus))
		 receive |= 1;
           delay_us(1); 	   
    }					 
    SCL_Output(bus,0);
    return receive;
}
//========================================================================
// 业务层读写函数
//========================================================================

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t data)
{				   	  	    																 
     IIC_Start(bus);               
	
	IIC_Send_Byte(bus,daddr<<1);	    
	if(IIC_Wait_Ack(bus))	//等待应答
	{
		IIC_Stop(bus);		 
		return 1;		
	}
	IIC_Send_Byte(bus,reg);
	IIC_Wait_Ack(bus);	   	 										  		   
	IIC_Send_Byte(bus,data);     						   
	IIC_Wait_Ack(bus);  		    	   
     IIC_Stop(bus);
	delay_us(1);
	return 0;
}

uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t length,uint8_t buff[])
{			
	unsigned char i;	
     IIC_Start(bus);               //发送起始信号
	
	IIC_Send_Byte(bus,daddr<<1);	//呼叫设备并声明要写，把七位的设备地址左移1位，最低位补0，代表写，发送设备地址+写指令     
	if(IIC_Wait_Ack(bus))          //看看传感器是否有回应，没回应报错退出
	{
		IIC_Stop(bus);
		return 1;
	}
	IIC_Send_Byte(bus,reg);        //指定房间号：发送寄存器地址（告诉传感器要写哪个配置）
	IIC_Wait_Ack(bus);	
	for(i=0;i<length;i++)        //多字节写就是在这发送数据的这步，比单字节写嵌套了一个for循环
	{
		IIC_Send_Byte(bus,buff[i]);     						   
		IIC_Wait_Ack(bus); 
	}		    	   
     IIC_Stop(bus);
	delay_us(1);
	return 0;
} 

//读操作分为假写和真正读取，因为假设要读温度的话，你得先告诉传感器要读哪个寄存器，这个过程就是得用写的方式告诉
unsigned char IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg)
{
	uint8_t dat;
	IIC_Start(bus);             //发送起始信号
	IIC_Send_Byte(bus,daddr<<1); //声明写，把七位的设备地址左移1位，最低位补0，代表写
	IIC_Wait_Ack(bus);
	IIC_Send_Byte(bus,reg);      //把想读的寄存器地址发过去（传感器的指针指到了这里），到这里都是假写阶段
	IIC_Wait_Ack(bus);
	
	IIC_Start(bus);             //真读阶段开始，重新发起起始信号，总线不断开
	IIC_Send_Byte(bus,(daddr<<1)+1);//最低位变成1代表读，左移1位之后，最低位强制加1
	IIC_Wait_Ack(bus);             //传感器回应说准备好发数据了
	dat = IIC_Receive_Byte(bus);   //开始听传感器发来的8位数据
	IICSend_NAck(bus);           //单片机发送NACK不应答，表示读完这一个就不读了，要求从机/传感器交出SDA的控制权
	IIC_Stop(bus);                 //结束通信
	return dat;                    //返回读到的数据
}

uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
	uint8_t i;
	IIC_Start(bus);
	IIC_Send_Byte(bus,daddr<<1);
	if(IIC_Wait_Ack(bus))
	{
		IIC_Stop(bus);		 
		return 1;		
	}
	IIC_Send_Byte(bus,reg);
	IIC_Wait_Ack(bus);
	
	IIC_Start(bus);                 //上面的假写阶段都一样，真读阶段就是从听单个8位数据变为连读听数据
	IIC_Send_Byte(bus,(daddr<<1)+1); 
	IIC_Wait_Ack(bus);
	for(i=0;i<length;i++)         //开始连续听数据
		{
		buff[i] = IIC_Receive_Byte(bus); 
		if(i<length-1)              
		{IICSend_Ack(bus);}           
	}
	IICSend_NAck(bus);             
	IIC_Stop(bus);                    
	return 0;
}
