#include "version.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  
/**
 * @brief 获取版本字符串
 */
const char *Version_GetString(void) { return APP_VERSION_STRING; }

/**
 * @brief 获取完整版本信息
 */
const char *Version_GetFullInfo(void) {
  static char full_info[64];
  snprintf(full_info, sizeof(full_info), "%s %s", APP_NAME, APP_VERSION_STRING);
  return full_info;
}

/**
 * @brief 获取版本号（编码为32位整数）
 */
uint32_t Version_GetCode(void) {
  return (APP_VERSION_MAJOR << 24) | (APP_VERSION_MINOR << 16) |
         (APP_VERSION_PATCH << 8);
}
