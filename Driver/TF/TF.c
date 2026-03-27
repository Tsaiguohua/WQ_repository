#include "TF.h"
#include "ff.h"          
#include <stdio.h>
#include <string.h>
#include "delay.h"

// 逻辑驱动器的工作区（必须全局生存期，不能放局部栈）
static FATFS fs;          

uint8_t TF_Hardware_Init(void) {
    FRESULT res;
    
    // 在挂载前稍微延时，等待SD卡上电稳定
    delay_ms(500); 
    
    // 挂载 SD 卡文件系统 
    // 参数: 逻辑驱动器号 "0:"，参数: 1 表示立即挂载并检查卡是否就绪
    res = f_mount(&fs, "0:", 1);
    
    if (res == FR_OK) {
        printf("[TF] FatFs successfully mounted on SD card.\r\n");
        return 1; // 成功
    } else {
        printf("[TF] Failed to mount FatFs, error code: %d\r\n", res);
        return 0; // 失败
    }
}

uint8_t TF_Append_String(const char* filename, const char* str) {
    FIL file;
    FRESULT res;
    UINT bw;
    
    // 打开文件：使用 FA_OPEN_ALWAYS (如果不存在则创建，存在则打开) + 写权限（FA_WRITE）
    // 注意：由于旧版 FatFs (R0.11 等) 不支持 FA_OPEN_APPEND，这里需要使用 lseek 来实现追加
    res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
    
    if (res != FR_OK) {
        printf("[TF] ERROR: Failed to open or create file %s (err: %d)\r\n", filename, res);
        return 0;
    }
    
    // 关键：将文件写指针移动到文件末尾，这样后来的写入就是追加了
    res = f_lseek(&file, f_size(&file));
    if (res != FR_OK) {
        printf("[TF] ERROR: Failed to seek to end of file %s (err: %d)\r\n", filename, res);
        f_close(&file);
        return 0;
    }
    
    // 写入传入的长字符串
    res = f_write(&file, str, strlen(str), &bw);
    if (res != FR_OK || bw != strlen(str)) {
        printf("[TF] ERROR: Failed to write to file %s (err: %d)\r\n", filename, res);
        // 出错也要安全关闭文件，否则会造成坏道或锁卡
        f_close(&file);
        return 0;
    }
    
    // 正常写入完成，关闭文件触发缓存落盘
    f_close(&file);
    // printf("[TF] Successfully appended data to %s\r\n", filename);
    return 1;
}

uint8_t TF_Force_CSV_Header(const char* filename, const char* header_str) {
    // 强制把表头追加到文件末尾作为开机和记录的标志
    return TF_Append_String(filename, header_str);
}

