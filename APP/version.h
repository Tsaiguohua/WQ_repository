#ifndef __VERSION_H
#define __VERSION_H

#include <stdint.h>  
/**
 * @file    version.h
 * @brief   APP版本管理头文件（双APP架构）
 * @details 定义固件版本号和编译目标，支持APP1/APP2编译配置
 *
 * 使用方法：
 *   1. APP1编译配置：
 *      - MDK工程设置 → Target Options → C/C++ → Preprocessor Symbols
 *      - Define: APP_TARGET=1 APP_VERSION_MAJOR=1 APP_VERSION_MINOR=0
 *      - 链接脚本：选择stm32f407_app1.sct
 *
 *   2. APP2编译配置：
 *      - Define: APP_TARGET=2 APP_VERSION_MAJOR=2 APP_VERSION_MINOR=0
 *      - 链接脚本：选择stm32f407_app2.sct
 */

// ========== 版本号定义（通过编译器宏定义）==========
#ifndef APP_VERSION_MAJOR
#define APP_VERSION_MAJOR 1 // 主版本号（默认1）
#endif

#ifndef APP_VERSION_MINOR
#define APP_VERSION_MINOR 1 // 次版本号（默认0）
#endif

#ifndef APP_VERSION_PATCH
#define APP_VERSION_PATCH 0 // 补丁版本号（默认0）
#endif

// ========== 编译目标（1=APP1, 2=APP2）==========
#ifndef APP_TARGET
#define APP_TARGET 1 // 默认编译APP1
#endif

// ========== 自动生成版本字符串 ==========
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define APP_VERSION_STRING                                                     \
  "V" TOSTRING(APP_VERSION_MAJOR) "." TOSTRING(                                \
      APP_VERSION_MINOR) "." TOSTRING(APP_VERSION_PATCH)

// ========== 完整版本信息 ==========
#if APP_TARGET == 1
#define APP_NAME "APP1 (Backup)"
#define APP_BASE_ADDRESS 0x08010000
#elif APP_TARGET == 2
#define APP_NAME "APP2 (Update)"
#define APP_BASE_ADDRESS 0x08090000
#else
#error "APP_TARGET must be 1 or 2"
#endif

// ========== 辅助函数声明 ==========
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取版本字符串
 * @return 版本字符串指针（如"V1.0.0"）
 */
const char *Version_GetString(void);

/**
 * @brief 获取完整版本信息（包含APP名称）
 * @return 完整版本信息（如"APP1 (Backup) V1.0.0"）
 */
const char *Version_GetFullInfo(void);

/**
 * @brief 获取版本号（编码为32位整数）
 * @return 版本号（格式：0xMMNNPPRR，MM=Major, NN=Minor, PP=Patch, RR=保留）
 */
uint32_t Version_GetCode(void);

#ifdef __cplusplus
}
#endif

#endif /* __VERSION_H */
