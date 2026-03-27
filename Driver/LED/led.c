#include "led.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//正点原子STM32F407开发板
//LED驱动代码	   
//作者：蔡国华
//创建日期:2025/12/23
//版本：V1.0								  
////////////////////////////////////////////////////////////////////////////////// 	 

//初始化PF9和PF10为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	//使能GPIOF时钟

	//GPIOF9,F10初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//LED0和LED1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);					//初始化GPIO
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);			//GPIOF9,F10设置高，灯灭

}






