/**
 * @file    WQInterface.c
 * @author  Antigravity Refactor Team
 * @brief   水质监测系统统一接口单例实现层
 * @version 2.0 (Architecture Upgrade B)
 * @date    2026-04-19
 *
 * @details 
 *   本文件实现了 WQInterface.h 中定义的单例对象。核心逻辑包括：
 *   1. 传感器函数指针映射表 (_sensor_map)：将传感器类型与底层驱动函数绑定。
 *   2. 通道绑定逻辑 (WQ_Interface_Bind)：支持动态发现并挂载传感器 ops。
 *   3. 线程安全保护：实现了 UART4 发送互斥锁，确保多任务并发下串口数据不交错。
 *   4. 资源初始化 (WQ_Interface_Init)：在任务运行前预创建系统级同步对象。
 */

#include "WQInterface.h"
#include "FreeRTOS.h"
#include "self_exam.h"
#include "cod.h"
#include "y4000.h"
#include "cdom.h"
#include "chl.h"
#include "do.h"
#include "ph.h"
#include "sal.h"
#include "voltage.h"
#include "oled.h"
#include "iic.h"
#include "gps.h"
#include "uart4.h"
#include "AHT20.h"
#include "buzzer.h"
#include "AcqTask.h"
#include "TF.h"
#include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h> 

/* UART4 专用的发送互斥锁，防止抢占导致串口数据交错 */
static SemaphoreHandle_t g_uart4_tx_mutex = NULL;

/**
 * @brief  WQInterface 初始化函数
 * @note   必须在 User_Tasks_Init() 最开头、任何任务创建之前调用。
 *         确保 g_uart4_tx_mutex 只在单一时机、由主流程创建，
 *         彻底避免 UpLoad_Task 与 Heartbeat_Task 同时首次调用
 *         WQ_UART4_Send() 时发生的双重创建竞态条件（内存泄漏 + 互斥保护失效）。
 */
void WQ_Interface_Init(void) {
    if (g_uart4_tx_mutex == NULL) {
        g_uart4_tx_mutex = xSemaphoreCreateMutex();
        if (g_uart4_tx_mutex == NULL) {
            /* 内存不足，互斥锁创建失败，打印告警（发送时会跳过锁保护） */
            printf("[WQInterface] ERROR: g_uart4_tx_mutex create failed!\r\n");
        } else {
            printf("[WQInterface] g_uart4_tx_mutex created OK.\r\n");
        }
    }
}


/* =========================================================================
 * 辅助函数：将传感器读数压入系统的 KV 数组中
 * ========================================================================= */
static void Push_KVNode(acquisition_data_t *acq, const char *key, float value) {
    if (acq->current_node_count < MAX_WATER_PARAMS) {
        strncpy(acq->water_nodes[acq->current_node_count].key, key, 11);
        acq->water_nodes[acq->current_node_count].key[11] = '\0';
        acq->water_nodes[acq->current_node_count].value = value;
        acq->water_nodes[acq->current_node_count].is_valid = true;
        acq->current_node_count++;
    }
}

#if WQ_USE_SENSOR_COD
static uint8_t WQ_COD_GetData(void *out) {
    cod_data_t raw;
    acquisition_data_t *acq = (acquisition_data_t *)out;
    if (cod_read(&raw)) {
        Push_KVNode(acq, "WTemp", raw.temp);
        Push_KVNode(acq, "COD", raw.cod);
        Push_KVNode(acq, "TOC", raw.toc);
        Push_KVNode(acq, "TUR", raw.tur);
        return 1;
    }
    return 0;
}
#endif

#if WQ_USE_SENSOR_Y4000
static uint8_t WQ_Y4000_GetData(void *out) {
    y4000_data_t raw;
    acquisition_data_t *acq = (acquisition_data_t *)out;
    if (y4000_read(&raw)) {
        Push_KVNode(acq, "Y_PH", raw.ph_value);
        Push_KVNode(acq, "Y_DO", raw.do_value);
        Push_KVNode(acq, "Y_SAL", raw.sal_value);
        Push_KVNode(acq, "Y_ATM", raw.atm_value);
        return 1;
    }
    return 0;
}
#endif

#if WQ_USE_SENSOR_CDOM
static uint8_t WQ_CDOM_GetData(void *out){
	float cdom_val;
	if(cdom_read(&cdom_val)){
		Push_KVNode((acquisition_data_t*)out, "CDOM", cdom_val);
		return 1;
	}
	return 0;
}
#endif

#if WQ_USE_SENSOR_CHL
static uint8_t WQ_CHL_GetData(void *out){
	float chl_val;
	if(chl_read(&chl_val)){
		Push_KVNode((acquisition_data_t*)out, "CHL", chl_val);
		return 1;
	}
	return 0;
}
#endif

#if WQ_USE_SENSOR_PH
static uint8_t WQ_PH_GetData(void *out){
	float ph_val;
	if(ph_read(&ph_val)){
		Push_KVNode((acquisition_data_t*)out, "PH", ph_val);
		return 1;
	}
	return 0;
}
#endif

#if WQ_USE_SENSOR_DO
static uint8_t WQ_DO_GetData(void *out){
	float do_val;
	if(do_read(&do_val)){
		Push_KVNode((acquisition_data_t*)out, "DO", do_val);
		return 1;
	}
	return 0;
}
#endif

#if WQ_USE_SENSOR_SAL
static uint8_t WQ_SAL_GetData(void *out){
	float sal_val;
	if(sal_read(&sal_val)){
		Push_KVNode((acquisition_data_t*)out, "SAL", sal_val);
		return 1;
	}
	return 0;
}
#endif

#if WQ_USE_AHT20
static iic_bus_t AHT_bus = 
{
	.SDA_PORT = GPIOF,
	.SCL_PORT = GPIOF,
	.SDA_PIN  = GPIO_Pin_2,
	.SCL_PIN  = GPIO_Pin_3,
	.RCC_AHB1Periph = RCC_AHB1Periph_GPIOF
};

static AHT20_t dev = {
	.bus = &AHT_bus,
	.addr_write = 0x70,
	.addr_read = 0x71,
	.calibrated = false
};
#endif

uint8_t WQ_AHT20_Init(void){
#if WQ_USE_AHT20
    bool ret = aht20_init(&dev);
    return ret ? 1 : 0;
#else
    return 0; 
#endif
}

uint8_t WQ_AHT20_Get_Humi_Temp(float *temperature, float *humidity){
#if WQ_USE_AHT20
    bool ret = aht20_read_temp_humi(&dev, temperature, humidity);
    return ret ? 1 : 0;
#else	
    *temperature = 0.0f;
    *humidity = 0.0f;
    return 0; 
#endif
}
uint8_t WQ_Battery_Init(void)
{
#if WQ_USE_Battery
	Voltage_Init();
	return 1;
#else
	return 0;
#endif
}


uint8_t WQ_Get_Battery(void)
{  
#if WQ_USE_Battery
	Voltage_GetBattery();
	return 1;
	
#else
	return 0;
#endif
}



uint8_t WQ_UART4_Send(const char *str)
{
#if WQ_USE_NetWork 
	if(str == NULL)
		return 0;

    /* 获取发送锁，防止被其他任务抢占插嘴（锁已在 WQ_Interface_Init() 中提前创建） */
    if (g_uart4_tx_mutex != NULL) {
        xSemaphoreTake(g_uart4_tx_mutex, portMAX_DELAY);
    }

    /* ⚠️ 关键修复：发送前临时关闭 IDLE 中断
     * 防止发送操作触发虚假的 IDLE 中断，导致 DMA 接收通道被 DeInit 摧毁 */
    USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);

	const char *p = str;
	while(*p){
		while (USART_GetFlagStatus(UART4 ,USART_FLAG_TXE) == RESET);
		USART_SendData(UART4,(uint8_t)*p);
		p++;
	}
	while (USART_GetFlagStatus (UART4,USART_FLAG_TC) == RESET);

    /* 发送完毕，清除可能残留的 IDLE 标志位，再重新使能 IDLE 中断 */
    {
        volatile uint32_t __sr = UART4->SR;
        volatile uint32_t __dr = UART4->DR;
        (void)__sr; (void)__dr;
    }
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    /* 释放锁 */
    if (g_uart4_tx_mutex != NULL) {
        xSemaphoreGive(g_uart4_tx_mutex);
    }

	return 1;
#endif
}

uint8_t WQ_GPS_Init(void (*rx_callback)(void))
{
#if WQ_USE_GPS
    GPS_Init(rx_callback);
	return 1;
#else
    return 0;     
#endif
}

bool WQ_GPS_GetRawFrame(char *buf, uint16_t max_len)
{
#if WQ_USE_GPS
    return GPS_GetRawFrame(buf, max_len);
#else
    return false;
#endif
}

bool WQ_GPS_ParseNMEA(const char *frame_buf, gps_data_t *parsed_data)
{
#if WQ_USE_GPS
    return GPS_ParseNMEA(frame_buf, parsed_data);
#else
    return false;
#endif
}

extern bool GPSTask_GetData(gps_data_t *data);

uint8_t WQ_Get_GPS(gps_data_t *data)
{
    if (data == NULL) {
        return 0; 
    }
#if WQ_USE_GPS
    bool ret = GPSTask_GetData(data);
    return ret ? 1 : 0;
#else
    data->latitude = 0.0f;
    data->longitude = 0.0f;
    data->is_valid = false;
    return 0;     
#endif
}


uint8_t WQ_OLED_Init(void)
{
#if WQ_USE_Display
	OLED_Init();
	return 1;
#else
    return 0;  
#endif
}

uint8_t WQ_OLED_Clear(void)
{
#if WQ_USE_Display
	OLED_Clear();
	return 1;
#else
    return 0;  
#endif
}


uint8_t WQ_OLED_ShowBootAnimation(void)
{
#if WQ_USE_Display
	OLED_ShowBootAnimation();
	return 1;
#else
    return 0;  
#endif
}


uint8_t WQ_OLED_ShowMainScreen(const char *year, const char *time, const char *battery, const char *temp, const char *freq, const char *hum, uint8_t ch1, uint8_t ch2, uint8_t ch3)
{
#if WQ_USE_Display
	OLED_ShowMainScreen(year, time, battery, temp, freq, hum, ch1, ch2, ch3);
	return 1;
#else
    return 0;  
#endif
}

uint8_t WQ_Storage_Init(void) {
#if WQ_USE_TF
    return TF_Hardware_Init();
#else
    return 0;  	
#endif
}

uint8_t WQ_Storage_WriteCSVHeader(const char *header_str) {
#if WQ_USE_TF
    return TF_Force_CSV_Header("0:Record.csv", header_str);
#else
    return 0;
#endif
}

uint8_t WQ_Storage_WriteCSV(const char *csv_str) {
#if WQ_USE_TF
    return TF_Append_String("0:Record.csv", csv_str);
#else
    return 0; 
#endif	
}

uint8_t WQ_Storage_WriteTXT(const char *txt_str) {
#if WQ_USE_TF	
    return TF_Append_String("0:Detail.txt", txt_str);
#else
    return 0; 
#endif	
}



void WQ_System_Set4GPower(bool on)
{
#if WQ_USE_NetWork	
    SelfExam_Set4GPower(on);
#endif
}
void WQ_System_Restart(void)
{
    NVIC_SystemReset();
}

void WQ_System_SetBuzzer(bool on)
{
    if (on) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }
}



void WQ_System_ScanAndBindChannels(void)
{
    uint8_t address = 0;
    
    printf("\r\n========================================\r\n");
    printf("       传感器总线扫描与通道绑定开始\r\n");
    printf("========================================\r\n");

    /* 确保初始状态为全部断开 */
    SelfExam_CloseAllChannels();

    for (int i = 0; i < 3; i++) {
        int ch = i + 1;
        SelfExam_OpenChannel(ch);
        /* 等待高侧开关和传感器上电稳定 */
        vTaskDelay(pdMS_TO_TICKS(1500));

        address = SelfExam_ProbeAddress();

        /* 如果探测到传感器，将其类型绑定到中间层的内部台账 */
        if (address != 0) {
           WQInterface.Channel[i].type = (sensor_type_t)address;
           printf("[Scan] Channel %d Detected: %d (Addr=0x%02X)\r\n", ch, address, address);
        } else {
           WQInterface.Channel[i].type = SENSOR_TYPE_NONE;
           printf("[Scan] Channel %d No sensor detected\r\n", ch);
        }

        SelfExam_CloseChannel(ch);
    }
    
    /* 调用自身的 Bind 方法生成函数指针挂载 */
    WQ_Interface_Bind();

    printf("========================================\r\n");
    printf("       传感器扫描与绑定完成\r\n");
    printf("========================================\r\n\r\n");
}




WQ_InterfaceTypeDef WQInterface = {
	.Battery = {
		.Init = WQ_Battery_Init,
		.GetBattery = WQ_Get_Battery
	},
	.AHT20 = {
		.Init = WQ_AHT20_Init,
		.GetHumiTemp = WQ_AHT20_Get_Humi_Temp
	},
	.Network = {
		.connected = 1,
		.Send = WQ_UART4_Send
	},
	.GPS = {
		.Init = WQ_GPS_Init,
		.GetRawFrame = WQ_GPS_GetRawFrame,
		.ParseNMEA = WQ_GPS_ParseNMEA,
		.GetGPSData = WQ_Get_GPS 
	},
	.System ={                          
        .Restart = WQ_System_Restart,
        .SetBuzzer = WQ_System_SetBuzzer,
        .ScanAndBindChannels = WQ_System_ScanAndBindChannels,
        .Set4GPower = WQ_System_Set4GPower
	},
	.Display = {

		.Init = WQ_OLED_Init,
		.Clear = WQ_OLED_Clear,
		.ShowBootAnimation = WQ_OLED_ShowBootAnimation,
		.ShowMainScreen = WQ_OLED_ShowMainScreen

	},
	.Storage = {
		.Init = WQ_Storage_Init,
		.WriteCSVHeader = WQ_Storage_WriteCSVHeader,
		.WriteCSV = WQ_Storage_WriteCSV,
		.WriteTXT = WQ_Storage_WriteTXT
	}
};

// 传感器类型 → wrapper 函数的查找表
typedef struct {
    sensor_type_t  type;
    const char    *name;              // 传感器名称，绑定时赋给 Channel.name
    uint8_t      (*read_fn)(void *out);
    void         (*power_on)(void);   // 上电预热（暂为 NULL，低功耗时填入）
    void         (*power_off)(void);  // 下电（暂为 NULL，低功耗时填入）
} _SensorMap_t;

static const _SensorMap_t _sensor_map[] = {
    /*  type                 name      read_fn              power_on  power_off */
    { SENSOR_TYPE_COD,   "COD",   WQ_COD_GetData,   NULL, NULL },
    { SENSOR_TYPE_Y4000, "Y4000", WQ_Y4000_GetData, NULL, NULL },
    { SENSOR_TYPE_PH,    "PH",    WQ_PH_GetData,    NULL, NULL },
    { SENSOR_TYPE_DO,    "DO",    WQ_DO_GetData,    NULL, NULL },
    { SENSOR_TYPE_SAL,   "SAL",   WQ_SAL_GetData,   NULL, NULL },
    { SENSOR_TYPE_CDOM,  "CDOM",  WQ_CDOM_GetData,  NULL, NULL },
    { SENSOR_TYPE_CHL,   "CHL",   WQ_CHL_GetData,   NULL, NULL },
};

void WQ_Interface_Bind(void) {
    for (int i = 0; i < 3; i++) {
        sensor_type_t current_type = WQInterface.Channel[i].type;

        /* 先清空本通道的所有字段 */
        WQInterface.Channel[i].channel   = i + 1;
        WQInterface.Channel[i].Read      = NULL;
        WQInterface.Channel[i].name      = "NONE";
        WQInterface.Channel[i].power_on  = NULL;
        WQInterface.Channel[i].power_off = NULL;
        WQInterface.Channel[i].connected = 0;

        if (current_type == SENSOR_TYPE_NONE) continue;

        for (int j = 0; j < (int)(sizeof(_sensor_map)/sizeof(_sensor_map[0])); j++) {
            if (_sensor_map[j].type == current_type) {
                WQInterface.Channel[i].Read      = _sensor_map[j].read_fn;
                WQInterface.Channel[i].name      = _sensor_map[j].name;
                WQInterface.Channel[i].power_on  = _sensor_map[j].power_on;
                WQInterface.Channel[i].power_off = _sensor_map[j].power_off;
                WQInterface.Channel[i].connected = 1;
                printf("[Bind] Channel %d -> %s\r\n", i + 1, _sensor_map[j].name);
                break;
            }
        }
    }
}


