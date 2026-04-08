#include "FreeRTOS.h"
#include "TasksInit.h"   
#include "delay.h"
#include "flash.h"
#include "iic.h"
#include "led.h"
#include "oled.h"
#include "self_exam.h"
#include "sys.h"
#include "task.h"
#include "uart4.h" 
#include "usart.h"
#include "watchdog.h" // 看门狗驱动

//////////////////////////////////////////////////////////////////////////////////
// 水质监测系统主程序 - FreeRTOS版本
//
// 硬件平台：正点原子 STM32F407ZGT6 最小系统板
//   - MCU：STM32F407ZGT6 (ARM Cortex-M4, 168MHz)
//   - Flash：1MB
//   - RAM：192KB (128KB + 64KB CCM)
//
// 系统架构：
//   - 操作系统：FreeRTOS v9.0.0
//   - 任务总数：8个（初始化、采集、上传、命令、OLED、APP、GPS、看门狗）
//   - 通信接口：
//     * USART1 (PA9/PA10)：调试串口，115200波特率
//     * UART4 (PC10/PC11)：4G模块MQTT通信，115200波特率
//     * I2C (软件模拟，PD0/PD1)：OLED显示、传感器通信
//
// 启动流程：
//   1. 硬件底层初始化
//      - NVIC中断优先级分组（Group 4）
//      - 延时函数初始化（用于FreeRTOS前的延时）
//      - USART1调试串口初始化
//      - LED指示灯初始化
//      - IIC总线初始化（软件I2C）
//
//   2. Flash配置管理
//      - Flash_Init()：创建Flash互斥锁
//      - Flash_LoadSystemState()：从Flash恢复系统配置
//        * 通道状态（3个通道开关状态）
//        * 采集频率（默认5秒）
//        * 上传频率（默认10秒）
//
//   3. 通信模块初始化
//      - UART4_MQTT_Init()：初始化4G模块MQTT通信
//
//   4. 创建初始化任务
//      - app_init_task：优先级9（最高），堆栈768字（3072B）
//      - 此任务负责创建所有业务任务后自删除
//
//   5. 启动FreeRTOS调度器
//      - vTaskStartScheduler()：永不返回
//
// 任务优先级分配（数字越大优先级越高）：
//   ┌─────────────────────────────────────────────────────────┐
//   │ 优先级 │ 任务名称        │ 堆栈(字) │ 周期/触发     │
//   ├─────────────────────────────────────────────────────────┤
//   │   9    │ app_init_task   │   768    │ 一次性（自删除）│
//   │   7    │ Command_Task    │   512    │ 信号量触发     │
//   │   6    │ Acquisition_Task│   768    │ 5秒（可配置）  │
//   │   5    │ app_task        │   384    │ 4秒           │
//   │   4    │ app_oled_task   │   640    │ 1秒           │
//   │   3    │ Upload_Task     │   768    │ 10秒（可配置） │
//   │   2    │ Heartbeat_Task  │   512    │ 240秒（固定）  │
//   │   2    │ GPS_Task        │   512    │ 事件触发       │
//   │   1    │ Watchdog_Task   │   192    │ 15秒          │
//   └─────────────────────────────────────────────────────────┘
//
// 任务功能说明：
//   - app_init_task：创建所有业务任务，初始化OLED硬件
//   - Command_Task：处理MQTT命令，修改系统配置并保存到Flash
//   - Acquisition_Task：采集水质传感器数据（COD、CDOM、PH等）
//   - app_task：采集环境数据（AHT20温湿度、电池电量）
//   - app_oled_task：更新OLED显示（温湿度、频率、电量等）
//   - Upload_Task：打包JSON数据并通过4G上传到MQTT服务器
//   - Heartbeat_Task：每240秒发送心跳消息，证明设备在线
//   - GPS_Task：解析GPS数据获取经纬度和时间
//   - Watchdog_Task：定期喂狗，防止系统死锁
//
// Flash持久化数据：
//   - 通道1/2/3状态（开启/关闭）
//   - 采集频率（1~3600秒）
//   - 上传频率（10~3600秒）
//   修改后自动保存，重启后恢复
//
// 注意事项：
//   ⚠️ OLED_Init()不能在main()中调用，必须在FreeRTOS任务中初始化
//   ⚠️ U8g2库堆栈消耗>4KB，主栈无法承受，需在任务中初始化
//   ⚠️ Flash_LoadSystemState()在调度器启动前调用，不使用FreeRTOS互斥锁
//   ⚠️ 所有业务任务由app_init_task统一创建，确保初始化顺序正确
//
// 作者：蔡国华
// 创建日期：2025/12/30
// 最后更新：2026/01/12
// 版本：v1.2
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  主函数
 *
 * 任务调度流程：
 * 1. 硬件初始化 (SysTick, UART, I2C, LED)
 * 2. Flash参数加载
 * 3. 硬件模块初始化 (W25Q128, OTA, UART4)
 * 4. 创建初始化任务 (app_init_task)
 * 5. 启动调度器
 */
int main(void) {
  // ⚠️ 关键：设置中断向量表偏移 (APP2 = 0x08090000)
  // 必须在任何中断（包括SysTick）启用前设置！
  SCB->VTOR = 0x08080000;

  /* 设置NVIC中断分组为 Group 4 */
  /* 抢占优先级 0-15，子优先级 0 */
  // 注意：FreeRTOS通常建议使用 Group 4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 设置系统中断优先级分组4
  delay_init(168);                                // 初始化延时函数
  uart_init(115200); // 初始化串口2，负责rs485和传感器通信
  LED_Init();        // 初始化LED端口

  // printf("[MAIN] Basic hardware initialized\r\n");

  //IIC_Init(); // 第1步测试：初始化软件I2C（PF2/PF3，用于AHT20）
  // printf("[MAIN] IIC initialized\r\n");
  // 面包板测试时暂时注释：初始化按键

  /* Flash模块初始化（创建互斥锁） */
  Flash_Init();

  // 擦除系统状态Flash区域，恢复初始状态
  // Flash_EraseSystemState();

  /* 从Flash读取之前保存的系统状态，恢复到内存变量中 */
  Flash_LoadSystemState();

  /* ============== OTA模块初始化 ============== */
  extern void W25Q128_Init(void);
  extern void OTA_Init(void);
  extern void OTA_ConfirmStartup(void);

  W25Q128_Init();       // 初始化W25Q128 Flash
  OTA_Init();           // 创建OTA互斥锁
  OTA_ConfirmStartup(); // 清零启动计数器，告诉Bootloader "APP正常运行"
  /* ========================================== */

  /* 初始化UART4 MQTT接收 */
  UART4_MQTT_Init();
  // printf("[MAIN] UART4 initialized for 4G module\r\n");

  /* 初始化独立看门狗（在FreeRTOS启动前） */
  // Watchdog_Init();y

  // printf("[MAIN] Creating app_init task...\r\n");

  /* ★ 新架构：统一任务初始化（替代原 app_init_task）
   *   - 创建 HardwareInitTask（最高优先级，负责硬件初始化后自删除）
   *   - 创建所有业务任务（采集、上传、OLED、命令等）
   */
  User_Tasks_Init();

  // printf("[MAIN] Starting FreeRTOS scheduler...\r\n");

  /* 启动FreeRTOS调度器 */
  vTaskStartScheduler();

  /* 正常情况下不会运行到这里 */
  while (1) {
    ; // 代码不应该运行到这里
  }
}

/*============================================================================
 *                          FreeRTOS钩子函数
 *===========================================================================*/

/**
 * @brief  堆栈溢出钩子函数
 * @note   当启用configCHECK_FOR_STACK_OVERFLOW时,如果检测到堆栈溢出会调用此函数
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  /* 使用简单的字符串输出，避免printf本身触发堆栈问题 */
  if (pcTaskName != NULL && pcTaskName[0] != '\0') {
    printf("[FATAL] Stack overflow in: %s\r\n", pcTaskName);
  } else {
    printf("[FATAL] Stack overflow in unknown task!\r\n");
  }

  /* 堆栈溢出后系统无法正常运行，停止在这里 */
  while (1) {
    ; // 堆栈溢出，停止运行
  }
}
