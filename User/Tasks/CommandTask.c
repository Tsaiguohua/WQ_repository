/**
 * @file    CommandTask.c
 * @author  Antigravity Refactor Team
 * @brief   MQTT 命令解析与指令分发任务 (Table-Driven)
 * @version 2.0 (Architecture Upgrade A)
 * @date    2026-04-19
 *
 * @details 
 *   本模块负责通过 UART4 接收来自 MQTT 服务器的 JSON 格式指令并执行。
 *   - 协议解析：采用“表驱动 (Table-Driven)”模式，避免了冗长的 if-else 链。
 *   - 模块化处理：每个指令（如 AcqFreq, Reset, OTA）都有对应的处理函数。
 *   - 稳健性：集成了串口 DMA 状态自动检查，防止指令接收队列意外卡死。
 */

#include "CommandTask.h"
#include "TasksInit.h"
#include "FreeRTOS.h"
#include "AcqTask.h"
#include "buzzer.h"
#include "flash.h" // Flash状态存储
#include "queue.h"
#include "self_exam.h"
#include "semphr.h" // 信号量支持
#include "task.h"
#include "uart4.h" // UART4 MQTT驱动
#include "UpLoadTask.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "WQInterface.h"
#include "cJSON.h" // 引入 cJSON 库
// 通信接口：
//   - 接收接口：UART4 (4G模块透传MQTT消息)
//   - 触发机制：UART中断接收到'{'和'}'，发送信号量通知Command任务
//
// 支持的命令：
//   1. 频率控制：
//      - {"AcqFrequency": 15}      设置采集频率（秒）
//      - {"UploadFrequency": 30}   设置上传频率（秒）
//      - {"Frequency": 20}         同时设置采集和上传频率（向后兼容）
//
//   2. 通道控制：
//      - {"ONECHANNEL_OPEN"}       开启通道1
//      - {"ONECHANNEL_CLOSE"}      关闭通道1
//      - {"TWOCHANNEL_OPEN"}       开启通道2
//      - {"TWOCHANNEL_CLOSE"}      关闭通道2
//      - {"THREECHANNEL_OPEN"}     开启通道3
//      - {"THREECHANNEL_CLOSE"}    关闭通道3
//      - {"ALL_CHANNEL_OPEN"}      开启所有通道
//      - {"ALL_CHANNEL_CLOSE"}     关闭所有通道
//
//   3. 数据获取：
//      - {"GET_MMSG"}              立即上传设备信息
//      - {"GET_DMSG"}              立即上传水质数据
//
//   4. 系统控制：
//      - {"RESTART"}               系统复位
//      - {"BUZZER_OPEN"}           开启蜂鸣器
//      - {"BUZZER_CLOSE"}          关闭蜂鸣器
//
// 命令处理流程：
//   1. UART4中断接收到完整JSON → 释放二值信号量
//   2. Command_Task被唤醒 → 解析命令字符串
//   3. 调用相应模块的API（Acquisition/Upload/etc）
//   4. 自动调用Flash_SaveSystemState()保存配置
//   5. 发送响应消息到MQTT服务器
//
// 自动Flash保存：
//   修改以下配置时自动保存到Flash，重启后恢复：
//   - 采集频率
//   - 上传频率
//
// 注意事项：
//   ⚠️ Command任务优先级较高（7），确保及时响应命令
//   ⚠️ Flash保存操作耗时约几十毫秒，会阻塞任务
//   ⚠️ 避免频繁发送命令，Flash擦写寿命有限
//
// 作者：蔡国华
// 创建日期：2026/01/04
// 最后更新：2026/01/11
// 版本：v1.0
//////////////////////////////////////////////////////////////////////////////////

/*============================================================================
 *                          全局变量
 *===========================================================================*/

/* UART接收消息队列（双缓冲机制使用，引自 TasksInit 模块）*/
extern QueueHandle_t xUartRxQueue;

/* 最后一次指令执行状态 */
static char last_command_status[50] = "WAITING_COMMAND";

/*============================================================================
 *                          私有函数声明
 *===========================================================================*/

static bool Command_HandleOTA(const char *cmd_str);
static bool Command_HandleFrequency(const char *cmd_str);
static bool Command_HandleAcqFrequency(const char *cmd_str);
static bool Command_HandleUploadFrequency(const char *cmd_str);
static bool Command_HandleGetData(const char *cmd_str);
static bool Command_HandleChannelControl(const char *cmd_str);
static bool Command_HandleSystemControl(const char *cmd_str);
static void Command_SendResponse(const char *response);

/*============================================================================
 *                          命令分发表（表驱动核心）
 *
 *  规则：
 *    1. 表项按从上到下顺序匹配，第一个命中的 handler 被调用，随即返回。
 *    2. 长关键字必须排在其前缀之前（如 AcqFrequency 必须先于 Frequency），
 *       否则短词会提前命中。
 *    3. 新增命令：只需在此表里加一行 + 实现对应 handler，
 *       Command_Parse() 永远不用再改。
 *===========================================================================*/

typedef struct {
    const char *keyword;                  /* strstr 匹配关键字           */
    bool      (*handler)(const char *);   /* 命中后调用的处理函数        */
} cmd_entry_t;

static const cmd_entry_t s_cmd_table[] = {
    /* ── OTA 升级指令（最高优先级，必须排最前） ─────────────── */
    { "config,upcheck",   Command_HandleOTA            },
    { "config,upin",      Command_HandleOTA            },
    { "UPFILE:",          Command_HandleOTA            },
    { "config,upsta",     Command_HandleOTA            },

    /* ── 频率指令（长词在前，防止短词 Frequency 先命中） ─────── */
    { "AcqFrequency",     Command_HandleAcqFrequency   },
    { "UploadFrequency",  Command_HandleUploadFrequency},
    { "Frequency",        Command_HandleFrequency      },

    /* ── 数据获取指令 ────────────────────────────────────────── */
    { "GET_MMSG",         Command_HandleGetData        },
    { "GET_DMSG",         Command_HandleGetData        },

    /* ── 通道控制指令 ────────────────────────────────────────── */
    { "CHANNEL",          Command_HandleChannelControl },

    /* ── 系统控制指令 ────────────────────────────────────────── */
    { "RESTART",          Command_HandleSystemControl  },
    { "BUZZER",           Command_HandleSystemControl  },
    { "4GPOWER",          Command_HandleSystemControl  },
};

#define CMD_TABLE_SIZE  (sizeof(s_cmd_table) / sizeof(s_cmd_table[0]))

/*============================================================================
 *                          API实现
 *===========================================================================*/

/**
 * @brief  初始化指令解析模块
 */
void Command_Init(void) { strcpy(last_command_status, "WAITING_COMMAND"); }

/**
 * @brief  解析并执行MQTT指令
 */
bool Command_Parse(const char *cmd_str, uint16_t len) {
    if (cmd_str == NULL || len == 0) {
        return false;
    }

    /* 遍历命令表，第一个关键字命中即调用对应 handler 并返回 */
    for (int i = 0; i < (int)CMD_TABLE_SIZE; i++) {
        if (strstr(cmd_str, s_cmd_table[i].keyword) != NULL) {
            return s_cmd_table[i].handler(cmd_str);
        }
    }

    /* 未命中任何表项：未知指令 */
    Command_SendResponse("{\"Client_Command\":\"ERROR_COMMAND\"}");
    strcpy(last_command_status, "ERROR_COMMAND");
    return false;
}

/**
 * @brief  获取最后一次指令执行状态
 */
const char *Command_GetLastStatus(void) { return last_command_status; }

/**
 * @brief  Command任务函数（双缓冲机制）
 * @note   从队列接收消息,处理MQTT指令（不负责重启DMA）
 */
void Command_Task(void *pvParameters) {
  UartRxMsg_t rxMsg;
  static char local_buffer[MQTT_RX_BUF_SIZE]; // 本地缓冲区
  static uint32_t rx_count = 0;               // 调试计数器

  printf("[Command] Task started, waiting for commands from queue...\\r\\n");
  printf("[Command] Queue size: 5, Item size: %d bytes\\r\\n",
         sizeof(UartRxMsg_t));

  while (1) {
    /* 阻塞等待队列消息 */

    if (xQueueReceive(xUartRxQueue, &rxMsg, pdMS_TO_TICKS(3000)) == pdTRUE) {
      rx_count++;
      printf("[Command] ===== Queue message received! Count: %lu =====\r\n",
             rx_count);

      /* 复制数据到本地缓冲区(此时中断可能已经在填充另一个缓冲区) */
      uint16_t copy_len = (rxMsg.length < MQTT_RX_BUF_SIZE - 1)
                              ? rxMsg.length
                              : (MQTT_RX_BUF_SIZE - 1);
      memcpy(local_buffer, rxMsg.pBuffer, copy_len);
      local_buffer[copy_len] = '\0';

      /* 调试：打印接收到的数据（简洁版）*/
      printf("[Command] Received %d bytes: %.50s...\r\n", copy_len,
             local_buffer);

      /* 解析并执行指令 */
      Command_Parse(local_buffer, copy_len);

      /* 注意：不需要重启DMA！中断已经自动切换缓冲区并重启了 */
    } else {
      // 超时：打印DMA状态用于调试（已注释，防止刷屏影响正常日志查看）
       printf("[Command] Queue timeout, checking DMA status...\r\n");
       UART4_PrintDMAStatus();
    }
  }
}

/*============================================================================
 *                          私有函数实现
 *===========================================================================*/

/**
 * @brief  处理 OTA 升级指令
 * @note   OTA 命令携带实际二进制长度（len），此处通过 strlen 降级处理；
 *         真实二进制长度由 Command_Task 层的 copy_len 保证，handler 层
 *         仅负责把完整字符串交给 OTA 模块。
 */
static bool Command_HandleOTA(const char *cmd_str) {
    extern void OTA_ProcessMQTTCommand(const char *cmd, uint16_t len);
    OTA_ProcessMQTTCommand(cmd_str, (uint16_t)strlen(cmd_str));
    return true;
}

/**
 * @brief  处理频率控制指令（向后兼容裸机）
 * @note   同时设置采集频率和上传频率
 */
static bool Command_HandleFrequency(const char *cmd_str) {
  uint16_t freq = 0;

  /* 使用 cJSON 强类型解析 */
  cJSON *root = cJSON_Parse(cmd_str);
  if (root != NULL) {
    cJSON *item = cJSON_GetObjectItem(root, "Frequency");
    if (item != NULL) {
      freq = (uint16_t)item->valueint;
    }
    cJSON_Delete(root);
  }

  /* 如果 JSON 解析失败，降级为旧版字符串提取（兼容容错） */
  if (freq == 0) {
    const char *freq_pos = strstr(cmd_str, "Frequency");
    if (freq_pos != NULL) {
      freq_pos = strchr(freq_pos, ':');
      if (freq_pos != NULL) {
        freq = (uint16_t)atoi(freq_pos + 1);
      }
    }
  }

  if (freq > 0) {
    /* 同时设置采集频率和上传频率 */
    Acquisition_SetFrequency(freq);
    Upload_SetFrequency(freq);

    /* 保存到Flash（频率改变） */
    Flash_SaveSystemState();

    /* 发送响应 */
    char response[80];
    sprintf(response, "{\"Client_Command\":\"Frequency_Change_OK(%d)\"}", freq);
    Command_SendResponse(response);

    strcpy(last_command_status, "FREQ_CHANGE_OK");
    return true;
  }

  return false;
}

/**
 * @brief  处理采集频率控制指令（新增）
 */
static bool Command_HandleAcqFrequency(const char *cmd_str) {
  uint16_t freq = 0;

  /* 使用 cJSON 强类型解析 */
  cJSON *root = cJSON_Parse(cmd_str);
  if (root != NULL) {
    cJSON *item = cJSON_GetObjectItem(root, "AcqFrequency");
    if (item != NULL) {
      freq = (uint16_t)item->valueint;
    }
    cJSON_Delete(root);
  }

  /* 如果 JSON 解析失败，降级为旧版字符串提取（兼容容错） */
  if (freq == 0) {
    const char *freq_pos = strstr(cmd_str, "AcqFrequency");
    if (freq_pos != NULL) {
      freq_pos = strchr(freq_pos, ':');
      if (freq_pos != NULL) {
        freq = (uint16_t)atoi(freq_pos + 1);
      }
    }
  }

  if (freq > 0 && Acquisition_SetFrequency(freq)) {
    /* 保存到Flash */
    Flash_SaveSystemState();

    char response[80];
    sprintf(response, "{\"Client_Command\":\"AcqFrequency_Change_OK(%d)\"}",
            freq);
    Command_SendResponse(response);

    strcpy(last_command_status, "ACQ_FREQ_OK");
    return true;
  }

  return false;
}

/**
 * @brief  处理上传频率控制指令（新增）
 */
static bool Command_HandleUploadFrequency(const char *cmd_str) {
  uint16_t freq = 0;

  /* 使用 cJSON 强类型解析 */
  cJSON *root = cJSON_Parse(cmd_str);
  if (root != NULL) {
    cJSON *item = cJSON_GetObjectItem(root, "UploadFrequency");
    if (item != NULL) {
      freq = (uint16_t)item->valueint;
    }
    cJSON_Delete(root);
  }

  /* 如果 JSON 解析失败，降级为旧版字符串提取（兼容容错） */
  if (freq == 0) {
    const char *freq_pos = strstr(cmd_str, "UploadFrequency");
    if (freq_pos != NULL) {
      freq_pos = strchr(freq_pos, ':');
      if (freq_pos != NULL) {
        freq = (uint16_t)atoi(freq_pos + 1);
      }
    }
  }

  if (freq > 0 && Upload_SetFrequency(freq)) {
    /* 保存到Flash */
    Flash_SaveSystemState();

    char response[80];
    sprintf(response, "{\"Client_Command\":\"UploadFrequency_Change_OK(%d)\"}",
            freq);
    Command_SendResponse(response);

    strcpy(last_command_status, "UPLOAD_FREQ_OK");
    return true;
  }

  return false;
}

/**
 * @brief  处理数据获取指令
 */
static bool Command_HandleGetData(const char *cmd_str) {
  if (strstr(cmd_str, "GET_MMSG") != NULL) {
    /* 立即上传一次设备信息 */
    Upload_SendNow();
    strcpy(last_command_status, "GET_MMSG_OK");
    return true;
  }

  if (strstr(cmd_str, "GET_DMSG") != NULL) {
    /* 立即上传一次水质数据 */
    Upload_SendNow();
    strcpy(last_command_status, "GET_DMSG_OK");
    return true;
  }

  return false;
}

/**
* @brief  处理通道控制指令,不放进flash里面了
 */
static bool Command_HandleChannelControl(const char *cmd_str) {
  /* 单通道控制 */
  if (strstr(cmd_str, "ONECHANNEL_OPEN") != NULL) {
    Acquisition_SetChannelState(1, 1);
    Command_SendResponse("{\"Client_Command\":\"ONECHANNEL_OPEN\"}");
    strcpy(last_command_status, "OPEN_ONEC_OK");
    return true;
  }
  if (strstr(cmd_str, "ONECHANNEL_CLOSE") != NULL) {
    Acquisition_SetChannelState(1, 0);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"ONECHANNEL_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_ONEC_OK");
    return true;
  }

  if (strstr(cmd_str, "TWOCHANNEL_OPEN") != NULL) {
    Acquisition_SetChannelState(2, 1);
  //  Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"TWOCHANNEL_OPEN\"}");
    strcpy(last_command_status, "OPEN_TWOC_OK");
    return true;
  }
  if (strstr(cmd_str, "TWOCHANNEL_CLOSE") != NULL) {
    Acquisition_SetChannelState(2, 0);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"TWOCHANNEL_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_TWOC_OK");
    return true;
  }

  if (strstr(cmd_str, "THREECHANNEL_OPEN") != NULL) {
    Acquisition_SetChannelState(3, 1);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"THREECHANNEL_OPEN\"}");
    strcpy(last_command_status, "OPEN_THREEC_OK");
    return true;
  }
  if (strstr(cmd_str, "THREECHANNEL_CLOSE") != NULL) {
    Acquisition_SetChannelState(3, 0);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"THREECHANNEL_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_THREEC_OK");
    return true;
  }

  /* 所有通道控制 */
  if (strstr(cmd_str, "ALL_CHANNEL_OPEN") != NULL) {
    Acquisition_SetChannelState(1, 1);
    Acquisition_SetChannelState(2, 1);
    Acquisition_SetChannelState(3, 1);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"ALL_CHANNEL_OPEN\"}");
    strcpy(last_command_status, "OPEN_CHANNEL_OK");
    return true;
  }
  if (strstr(cmd_str, "ALL_CHANNEL_CLOSE") != NULL) {
    Acquisition_SetChannelState(1, 0);
    Acquisition_SetChannelState(2, 0);
    Acquisition_SetChannelState(3, 0);
 //   Flash_SaveSystemState(); // 保存通道状态
    Command_SendResponse("{\"Client_Command\":\"ALL_CHANNEL_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_CHANNEL_OK");
    return true;
  }

  return false;
}

/**
 * @brief  处理系统控制指令
 */
static bool Command_HandleSystemControl(const char *cmd_str) {
  if (strstr(cmd_str, "RESTART") != NULL) {
    Command_SendResponse("{\"Client_Command\":\"RESTART_OK\"}");
    vTaskDelay(pdMS_TO_TICKS(500)); 
    
    // 🌟 通过接口调用复位，不碰 NVIC_SystemReset
    if (WQInterface.System.Restart != NULL) {
        WQInterface.System.Restart(); 
    }
    return true;
  }

  if (strstr(cmd_str, "BUZZER_OPEN") != NULL) {
    // 🌟 通过接口打开蜂鸣器
    if (WQInterface.System.SetBuzzer != NULL) {
        WQInterface.System.SetBuzzer(true);
    }
    Command_SendResponse("{\"Client_Command\":\"BUZZER_OPEN\"}");
    strcpy(last_command_status, "OPEN_BUZZER_OK");
    return true;
  }
  
  if (strstr(cmd_str, "BUZZER_CLOSE") != NULL) {
    // 🌟 通过接口关闭蜂鸣器
    if (WQInterface.System.SetBuzzer != NULL) {
        WQInterface.System.SetBuzzer(false);
    }
    Command_SendResponse("{\"Client_Command\":\"BUZZER_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_BUZZER_OK");
    return true;
  }

  if (strstr(cmd_str, "4GPOWER_OPEN") != NULL) {
    // 🌟 通过接口开启4G模块电源
    if (WQInterface.System.Set4GPower != NULL) {
        WQInterface.System.Set4GPower(true);
    }
    Command_SendResponse("{\"Client_Command\":\"4GPOWER_OPEN\"}");
    strcpy(last_command_status, "OPEN_4GPOWER_OK");
    return true;
  }

  if (strstr(cmd_str, "4GPOWER_CLOSE") != NULL) {
    // 🌟 通过接口关闭4G模块电源
    if (WQInterface.System.Set4GPower != NULL) {
        WQInterface.System.Set4GPower(false);
    }
    Command_SendResponse("{\"Client_Command\":\"4GPOWER_CLOSE\"}");
    strcpy(last_command_status, "CLOSE_4GPOWER_OK");
    return true;
  }

  return false;
}

/**
 * @brief  发送响应到MQTT（通过UART4）
 */
static void Command_SendResponse(const char *response) {
  if (response == NULL || WQInterface.Network.Send == NULL) {
    return;
  }
    WQInterface.Network.Send(response);
    
    WQInterface.Network.Send("\r\n");
  
}
