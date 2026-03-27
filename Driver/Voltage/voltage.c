#include "voltage.h"
#include "stm32f4xx.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// 电压/电量检测驱动实现
//
// 功能说明：
//   通过ADC1_Channel_1采样电压检测模块输出
//   使用线性映射计算电池电量百分比
//   多次采样取平均，防止电量跳变
//
// 硬件配置：
//   - ADC: ADC1
//   - 通道: Channel_1
//   - 引脚: PA1
//
// 电量计算（线性映射）：
//   满电ADC值: 1301 → 100%
//   空电ADC值: 908  → 0%
//   电量% = (ADC - 908) / (1301 - 908) * 100
//
// 创建日期: 2025/12/31
// 作者: 蔡国华
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          硬件配置宏定义
 *===========================================================================*/

#define VOLTAGE_ADC ADC1
#define VOLTAGE_ADC_CHANNEL ADC_Channel_1
#define VOLTAGE_GPIO_PORT GPIOA
#define VOLTAGE_GPIO_PIN GPIO_Pin_1
#define VOLTAGE_GPIO_CLK RCC_AHB1Periph_GPIOA
#define VOLTAGE_ADC_CLK RCC_APB2Periph_ADC1

/* 电量映射参数（根据实际电池和分压电路调整） */
#define BATTERY_ADC_MAX 1301.0f // 满电时的ADC值
#define BATTERY_ADC_MIN 908.0f  // 空电时的ADC值

/* 采样平均次数 */
#define SAMPLE_COUNT 40

/*============================================================================
 *                          全局变量
 *===========================================================================*/

uint8_t battery = 0;             // 电量百分比
float voltage_percentage = 0.0f; // 电量百分比（浮点）

/* 私有变量 */
static uint32_t adc_sum = 0;     // ADC累加值
static uint8_t sample_count = 0; // 采样计数
static uint8_t last_battery = 0; // 上次电量（用于突变保护）
static bool first_read = true;   // 首次读取标志

/*============================================================================
 *                          ADC初始化
 *===========================================================================*/

/**
 * @brief  初始化电压检测ADC
 * @note   PA1配置为模拟输入，ADC1单次转换模式
 */
void Voltage_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  /* 使能时钟 */
  RCC_AHB1PeriphClockCmd(VOLTAGE_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(VOLTAGE_ADC_CLK, ENABLE);

  /* GPIO配置为模拟输入 */
  GPIO_InitStructure.GPIO_Pin = VOLTAGE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;     // 模拟模式
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不上下拉
  GPIO_Init(VOLTAGE_GPIO_PORT, &GPIO_InitStructure);

  /* ADC公共配置 */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;    // 独立模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // 预分频4
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1配置 */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;          // 单通道模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    // 单次转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge =
      ADC_ExternalTrigConvEdge_None;                     // 软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
  ADC_InitStructure.ADC_NbrOfConversion = 1;             // 1个通道
  ADC_Init(VOLTAGE_ADC, &ADC_InitStructure);

  /* 配置ADC通道 */
  ADC_RegularChannelConfig(VOLTAGE_ADC, VOLTAGE_ADC_CHANNEL, 1,
                           ADC_SampleTime_84Cycles);

  /* 使能ADC */
  ADC_Cmd(VOLTAGE_ADC, ENABLE);

  /* 初始化变量 */
  battery = 0;
  voltage_percentage = 0.0f;
  adc_sum = 0;
  sample_count = 0;
  first_read = true;
}

/*============================================================================
 *                          ADC读取
 *===========================================================================*/

/**
 * @brief  读取ADC原始值
 * @return ADC值（0-4095）
 */
uint16_t Voltage_GetADC(void) {
  /* 配置通道 */
  ADC_RegularChannelConfig(VOLTAGE_ADC, VOLTAGE_ADC_CHANNEL, 1,
                           ADC_SampleTime_84Cycles);

  /* 启动转换 */
  ADC_SoftwareStartConv(VOLTAGE_ADC);

  /* 等待转换完成 */
  while (ADC_GetFlagStatus(VOLTAGE_ADC, ADC_FLAG_EOC) == RESET)
    ;

  /* 返回ADC值 */
  return ADC_GetConversionValue(VOLTAGE_ADC);
}

/*============================================================================
 *                          电量计算
 *===========================================================================*/

/**
 * @brief  读取电量百分比
 * @return 电量百分比（0-100）
 * @note   多次采样取平均，并做突变保护
 */
uint8_t Voltage_GetBattery(void) {
  uint16_t adc_value;
  float average_adc;

  /* 读取ADC */
  adc_value = Voltage_GetADC();

  /* 首次读取，直接计算（快速显示） */
  if (first_read) {
    if (adc_value >= BATTERY_ADC_MAX) {
      voltage_percentage = 100.0f;
    } else if (adc_value <= BATTERY_ADC_MIN) {
      voltage_percentage = 0.0f;
    } else {
      voltage_percentage = (adc_value - BATTERY_ADC_MIN) /
                           (BATTERY_ADC_MAX - BATTERY_ADC_MIN) * 100.0f;
    }
    battery = (uint8_t)(voltage_percentage + 0.5f); // 四舍五入
    last_battery = battery;
    first_read = false;
    return battery;
  }

  /* 累加ADC值 */
  adc_sum += adc_value;
  sample_count++;

  /* 达到采样次数后计算平均值 */
  if (sample_count >= SAMPLE_COUNT) {
    /* 计算平均ADC */
    average_adc = (float)adc_sum / SAMPLE_COUNT;

    /* 线性映射计算电量 */
    if (average_adc >= BATTERY_ADC_MAX) {
      voltage_percentage = 100.0f;
    } else if (average_adc <= BATTERY_ADC_MIN) {
      voltage_percentage = 0.0f;
    } else {
      voltage_percentage = (average_adc - BATTERY_ADC_MIN) /
                           (BATTERY_ADC_MAX - BATTERY_ADC_MIN) * 100.0f;
    }

    /* 转换为整数（四舍五入） */
    battery = (uint8_t)(voltage_percentage + 0.5f);

    /* 突变保护：如果电量下降超过10%，认为是干扰，沿用旧值 */
    if (last_battery > battery && (last_battery - battery > 10)) {
      battery = last_battery;
    }

    /* 更新历史值 */
    last_battery = battery;

    /* 低电量报警 */
    if (battery <= 5) {
      printf("Warning: Low battery (%d%%)!\r\n", battery);
    }

    /* 复位计数器 */
    adc_sum = 0;
    sample_count = 0;
  }

  return battery;
}

/**
 * @brief  检查是否低电量
 * @return true:低电量（<10%）
 */
bool Voltage_IsLow(void) 
{ 
return (battery < 10); 
}
