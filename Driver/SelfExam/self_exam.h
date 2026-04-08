#ifndef SELF_EXAM_H
#define SELF_EXAM_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// 传感器自检底层模块 - 高侧开关与探测驱动
//
// 功能说明：
//   负责系统开机阶段通道的物理上电（通过高侧开关），发出 Modbus 嗅探广播。
//   仅提供纯硬件驱动能力，不再绑定传感器识别及其它系统层应用逻辑。
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  初始化底层高侧开关控制引脚 GPIO
 */
void SelfExam_Init(void);

/**
 * @brief  接通指定通道的高侧开关上电
 * @param  channel: 通道号(1-3)
 */
void SelfExam_OpenChannel(uint8_t channel);

/**
 * @brief  切断指定通道的高侧开关断电
 * @param  channel: 通道号(1-3)
 */
void SelfExam_CloseChannel(uint8_t channel);

/**
 * @brief  切断所有通道的高侧开关
 */
void SelfExam_CloseAllChannels(void);

/**
 * @brief  向当前已上电开启的通道线束内发送 Modbus 广播并获取传感器地址回复
 * @return 真实收到的传感器地址(0x01~0x07)，0表示无传感器或响应超时
 */
uint8_t SelfExam_ProbeAddress(void);

/**
 * @brief  控制4G模块电源
 * @param  on: true=通电, false=断电
 */
void SelfExam_Set4GPower(bool on);

#endif
