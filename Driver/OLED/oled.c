#include "oled.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdio.h>
#include <string.h>


//////////////////////////////////////////////////////////////////////////////////
// OLED显示驱动 - 基于u8g2库（FreeRTOS版本）
//
// 功能说明：
//   本模块使用u8g2图形库驱动SSD1306 OLED显示屏（128x64像素），通过软件I2C通信。
//   提供开机动画和主界面显示功能，实时显示温湿度、电量、时间、传感器状态等信息。
//
// 硬件配置：
//   【OLED显示屏】
//   - 型号：SSD1306（128x64单色OLED）
//   - 接口：I2C（软件模拟）
//   - I2C地址：0x3C（7位地址，u8g2库自动处理）
//   - 电源：3.3V或5V（根据模块规格）
//
//   【GPIO引脚】
//   - SCL（时钟）：PD0
//   - SDA（数据）：PD1
//   - GPIO模式：开漏输出（OD，Open-Drain）+ 内部上拉
//   - GPIO速度：50MHz
//
// I2C通信方式（与AHT20的区别）：
//   【OLED的I2C实现】
//   - 驱动方式：u8g2库的软件I2C（u8x8_byte_sw_i2c）
//   - GPIO模式：开漏输出（GPIO_OType_OD）
//   - 控制方式：通过u8g2回调函数直接控制GPIO电平
//   - 延时方式：u8g2库内部调用回调函数实现延时
//   - 优点：u8g2库已封装I2C时序，无需手动实现起始/停止条件
//   - 缺点：只能用于u8g2设备，不通用
//
//   【AHT20的I2C实现（iic.c）】
//   - 驱动方式：手动实现的软件I2C（iic.c）
//   - GPIO模式：推挽输出（GPIO_OType_PP）+ 动态切换方向
//   - 控制方式：通过SDA_OUT()和SDA_IN()宏切换输入/输出
//   - 延时方式：delay_us()精确延时
//   - 优点：通用，可用于任何I2C设备
//   - 缺点：需要手动实现完整I2C时序
//
//   【关键区别对比】
//   ┌──────────────┬─────────────────┬─────────────────┐
//   │   对比项     │  OLED (u8g2)    │  AHT20 (iic.c)  │
//   ├──────────────┼─────────────────┼─────────────────┤
//   │ GPIO模式     │ 开漏（OD）      │ 推挽（PP）      │
//   │ 方向切换     │ 不需要          │ SDA_OUT/IN宏    │
//   │ I2C实现      │ u8g2库封装      │ 手动实现        │
//   │ 引脚         │ PD0/PD1         │ PF2/PF3         │
//   │ 延时方式     │ 回调函数        │ delay_us()      │
//   │ 通用性       │ 仅u8g2设备      │ 任何I2C设备     │
//   └──────────────┴─────────────────┴─────────────────┘
//
//   【为什么OLED用开漏？】
//   - u8g2库设计为标准I2C：要求开漏+外部上拉
//   - 直接控制GPIO输出0/1，依靠上拉电阻拉高
//   - 符合I2C总线标准（多主机、线与逻辑）
//
//   【为什么AHT20用推挽？】
//   - 通过动态切换方向实现双向通信
//   - 发送时推挽输出，直接驱动高低电平（更快）
//   - 接收时切换为上拉输入，读取外部电平
//   - 更灵活，适合单主机应用
//
// u8g2图形库：
//   【核心概念】
//   - 帧缓冲模式（Full Buffer）：所有绘图操作在RAM缓冲区进行
//   - u8g2_ClearBuffer()：清空缓冲区
//   - u8g2_DrawXXX()：绘制图形到缓冲区
//   - u8g2_SendBuffer()：将缓冲区内容发送到OLED显示
//
//   【字体支持】
//   - u8g2_font_ncenB12_tf：大字体（标题用）
//   - u8g2_font_ncenB08_tf：中字体（正文用）
//   - u8g2_font_inb21_mf：超大字体（时间显示）
//   - u8g2_font_5x7_tf：小字体（状态栏）
//
//   【绘图函数】
//   - u8g2_DrawStr()：绘制字符串
//   - u8g2_DrawLine()：绘制直线
//   - u8g2_DrawRFrame()：绘制圆角矩形框
//   - u8g2_DrawRBox()：绘制填充圆角矩形
//
// 显示内容：
//   【开机动画】
//   - 标题："SHOU-HDY"
//   - 进度条：从0%到100%，带圆角矩形
//   - 动画时长：约3秒
//   - 调用函数：OLED_ShowBootAnimation()
//
//   【主界面布局】
//   ```
//   ┌─────────────────────────────┐
//   │ HELLO HDY           ┌─────┐ │  ← 顶部状态栏
//   │                     │ 80% │ │  ← 电量显示
//   │                     └─────┘ │
//   ├─────────────────────────────┤  ← 分割线
//   │ A                           │  ← 通道A状态
//   │ B       05:04               │  ← 通道B + 时间
//   │ C     2025-12-31            │  ← 通道C + 日期
//   │                             │
//   │ 25C         10s         60% │  ← 温度/频率/湿度
//   └─────────────────────────────┘
//   ```
//
//   【显示数据来源】
//   - 电量：Voltage模块的battery变量
//   - 温度/湿度：采集任务的g_latest_acquisition_data
//   - 上传频率：采集任务的upload_frequency
//   - 通道状态：onechannel_state, twochannel_state, threechannel_state
//   - 时间：暂时固定"05:04"，后续改为GPS时间
//   - 日期：暂时固定"2025-12-31"
//
// API接口：
//   - OLED_Init()：初始化OLED（GPIO + u8g2库）
//   - OLED_Clear()：清屏
//   - OLED_ShowBootAnimation()：显示开机动画
//   - OLED_ShowMainScreen(...)：显示主界面（接收所有参数）
//   - OLED_Update()：更新主界面（自动读取全局数据）
//
// 使用示例：
//   ```c
//   // 1. 在app_init_task中初始化（必须在FreeRTOS任务中）
//   OLED_Init();  // ⚠️ u8g2库消耗大量堆栈，不能在main()中调用
//   
//   // 2. 显示开机动画（只显示一次）
//   OLED_ShowBootAnimation();
//   
//   // 3. 在app_oled_task中定期刷新主界面（1秒周期）
//   while (1) {
//       OLED_Update();  // 自动读取温湿度、电量等数据并显示
//       vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒刷新一次
//   }
//   ```
//
// FreeRTOS集成：
//   【堆栈要求】
//   - OLED_Init()：需要至少3KB堆栈（u8g2库内部分配）
//   - app_init_task堆栈：768字（3072字节）
//   - app_oled_task堆栈：640字（2560字节）
//
//   【延时函数】
//   - u8g2_delay_milli：使用vTaskDelay（FreeRTOS延时）
//   - u8g2_delay_10micro：使用忙等待循环（约10μs @168MHz）
//   - u8g2_delay_100nano：使用__NOP()（一个NOP指令）
//
//   【数据同步】
//   - 通过g_acquisition_data_mutex互斥锁保护共享数据
//   - OLED_Update()获取锁后读取温湿度数据
//   - 超时时间：10ms（避免阻塞OLED刷新）
//
// u8g2回调函数：
//   【u8x8_gpio_and_delay_stm32】
//   - U8X8_MSG_GPIO_AND_DELAY_INIT：初始化GPIO
//   - U8X8_MSG_DELAY_MILLI：毫秒延时
//   - U8X8_MSG_DELAY_10MICRO：10微秒延时
//   - U8X8_MSG_DELAY_100NANO：100纳秒延时
//   - U8X8_MSG_GPIO_I2C_CLOCK：设置SCL电平
//   - U8X8_MSG_GPIO_I2C_DATA：设置SDA电平
//
// 注意事项：
//   ⚠️ OLED_Init()必须在FreeRTOS任务中调用，不能在main()中调用
//   ⚠️ u8g2库消耗大量堆栈，主栈(2KB)无法承受
//   ⚠️ OLED与AHT20使用不同的I2C实现，不能混用
//   ⚠️ OLED使用PD0/PD1，AHT20使用PF2/PF3，引脚不冲突
//   ⚠️ 开漏输出需要外部上拉电阻（OLED模块通常已集成）
//   ⚠️ 刷新频率不要太高（建议1秒），避免闪烁和浪费CPU
//   ⚠️ u8g2的I2C地址0x3C不需要手动左移（库内部处理）
//
// 调试技巧：
//   - 如果OLED不显示：
//     1. 检查电源（3.3V或5V，根据模块规格）
//     2. 检查I2C引脚连接（PD0=SCL，PD1=SDA）
//     3. 检查OLED模块的I2C地址（通常是0x3C）
//     4. 确认在FreeRTOS任务中调用OLED_Init()
//     5. 检查堆栈大小（app_init_task需要>=3KB）
//   - 如果显示乱码：
//     1. 检查u8g2字体是否正确初始化
//     2. 检查字符串编码（u8g2支持ASCII）
//     3. 检查缓冲区是否越界
//   - 如果显示闪烁：
//     1. 降低刷新频率（从1秒改为2秒）
//     2. 检查是否频繁调用OLED_Clear()
//
// 与裸机版本的对比：
//   【相同点】
//   - 使用相同的u8g2库
//   - 显示内容和布局一致
//   - GPIO配置相同
//
//   【不同点】
//   - 延时函数：裸机用delay_ms()，FreeRTOS用vTaskDelay()
//   - 初始化位置：裸机在main()，FreeRTOS在app_init_task()
//   - 刷新方式：裸机在main循环，FreeRTOS在app_oled_task()
//   - 数据读取：FreeRTOS使用互斥锁保护共享数据
//
// 作者：蔡国华
// 创建日期：2025/12/31
// 最后更新：2026/01/14
// 版本：v1.0（基于u8g2库，FreeRTOS版本）
//////////////////////////////////////////////////////////////////////////////////



/*============================================================================
 *                          硬件配置宏定义
 *===========================================================================*/

#define OLED_SCL_PORT GPIOD
#define OLED_SCL_PIN GPIO_Pin_0 // PD0
#define OLED_SDA_PORT GPIOD
#define OLED_SDA_PIN GPIO_Pin_1 // PD1
#define OLED_GPIO_CLK RCC_AHB1Periph_GPIOD

/*============================================================================
 *                          全局变量
 *===========================================================================*/

u8g2_t u8g2; // u8g2对象

/* COMMAND字符串 */
char *COMMAND = "HELLO HDY";

/*============================================================================
 *                          u8g2回调函数
 *===========================================================================*/

/**
 * @brief  u8g2 GPIO和延时回调函数
 * @note   适配STM32F407
 */
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                  void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    /* GPIO初始化 */
    {
      GPIO_InitTypeDef GPIO_InitStructure;
      RCC_AHB1PeriphClockCmd(OLED_GPIO_CLK, ENABLE);

      GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // 开漏输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
      GPIO_Init(OLED_SCL_PORT, &GPIO_InitStructure);

      GPIO_SetBits(OLED_SCL_PORT, OLED_SCL_PIN);
      GPIO_SetBits(OLED_SDA_PORT, OLED_SDA_PIN);
    }
    break;

  case U8X8_MSG_DELAY_MILLI:
    /* 延时ms */
    vTaskDelay(pdMS_TO_TICKS(arg_int));
    break;

  case U8X8_MSG_DELAY_10MICRO:
    /* 延时10us（用ms替代，FreeRTOS无法精确us延时） */
    {
      volatile uint32_t i;
      for (i = 0; i < 168; i++)
        ; // 约10us @168MHz
    }
    break;

  case U8X8_MSG_DELAY_100NANO:
    /* 延时100ns */
    __NOP();
    break;

  case U8X8_MSG_GPIO_I2C_CLOCK:
    /* 设置SCL */
    if (arg_int) {
      GPIO_SetBits(OLED_SCL_PORT, OLED_SCL_PIN);
    } else {
      GPIO_ResetBits(OLED_SCL_PORT, OLED_SCL_PIN);
    }
    break;

  case U8X8_MSG_GPIO_I2C_DATA:
    /* 设置SDA */
    if (arg_int) {
      GPIO_SetBits(OLED_SDA_PORT, OLED_SDA_PIN);
    } else {
      GPIO_ResetBits(OLED_SDA_PORT, OLED_SDA_PIN);
    }
    break;

  default:
    return 0;
  }
  return 1;
}

/*============================================================================
 *                          OLED初始化
 *===========================================================================*/

/**
 * @brief  初始化OLED
 */
void OLED_Init(void) {
  /* 初始化u8g2（SSD1306, 128x64, I2C） */
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_sw_i2c,
                                         u8x8_gpio_and_delay_stm32);

  /* 初始化显示 */
  u8g2_InitDisplay(&u8g2);

  /* 唤醒显示 */
  u8g2_SetPowerSave(&u8g2, 0);

  /* 清空缓冲区 */
  u8g2_ClearBuffer(&u8g2);
}

/*============================================================================
 *                          显示函数
 *===========================================================================*/

/**
 * @brief  清屏
 */
void OLED_Clear(void) {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
}

/**
 * @brief  显示开机进度条动画
 * @note   和裸机draw1()一致
 */
void OLED_ShowBootAnimation(void) {
  int i;
  char buff[20];

  u8g2_ClearBuffer(&u8g2);
  u8g2_FirstPage(&u8g2);

  do {
    /* 进度条从0到100% */
    for (i = 10; i <= 80; i += 2) {
      u8g2_ClearBuffer(&u8g2);

      /* 计算百分比 */
      sprintf(buff, "%d%%", (int)(i / 80.0 * 100));

      /* 显示标题 "SHOU-HDY" */
      u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tf);
      u8g2_DrawStr(&u8g2, 10, 32, "SHOU-HDY");

      /* 显示当前进度百分比 */
      u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tf);
      u8g2_DrawStr(&u8g2, 100, 49, buff);

      /* 绘制进度条（填充部分） */
      u8g2_DrawRBox(&u8g2, 16, 40, i, 10, 4);

      /* 绘制进度条（外框） */
      u8g2_DrawRFrame(&u8g2, 16, 40, 80, 10, 4);

      /* 发送到显示屏 */
      u8g2_SendBuffer(&u8g2);
    }

    /* 显示完成后延时 */
    vTaskDelay(pdMS_TO_TICKS(400));

  } while (u8g2_NextPage(&u8g2));

  u8g2_ClearBuffer(&u8g2);
}

/**
 * @brief  显示主界面
 * @note   和裸机draw2()一致
 */
void OLED_ShowMainScreen(const char *year, const char *time, const char *battery, const char *temp, const char *freq, const char *hum, uint8_t ch1, uint8_t ch2, uint8_t ch3) {
  u8g2_ClearBuffer(&u8g2);
  u8g2_FirstPage(&u8g2);

  do {
    /* 左侧：传感器通道状态（A、B、C） */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tf);
    if (ch1 == 1) {
      u8g2_DrawStr(&u8g2, 2, 24, "A");
    }
    if (ch2 == 1) {
      u8g2_DrawStr(&u8g2, 2, 38, "B");
    }
    if (ch3 == 1) {
      u8g2_DrawStr(&u8g2, 2, 52, "C");
    }

    /* 顶部右侧：电量框 */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tf);
    u8g2_DrawStr(&u8g2, 95, 10, battery);
    u8g2_DrawRFrame(&u8g2, 90, 0, 36, 12, 3);

    /* 顶部分割线 */
    u8g2_DrawLine(&u8g2, 0, 14, 128, 14);

    /* 中间：大字体时间显示 */
    u8g2_SetFont(&u8g2, u8g2_font_inb21_mf);
    u8g2_DrawStr(&u8g2, 15, 40, time);

    /* 中下：日期 */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tf);
    u8g2_DrawStr(&u8g2, 35, 52, year);

    /* 底部：温度、频率、湿度 */
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tf);
    u8g2_DrawStr(&u8g2, 0, 63, temp);
    u8g2_DrawStr(&u8g2, 55, 63, freq);
    u8g2_DrawStr(&u8g2, 105, 63, hum);

    /* 顶部左侧：COMMAND字符串 */
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 0, 10, COMMAND);

  } while (u8g2_NextPage(&u8g2));

  /* 清空缓冲区（和裸机一致） */
  u8g2_ClearBuffer(&u8g2);
}

