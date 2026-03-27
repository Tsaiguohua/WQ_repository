#ifndef COMMANDTASK_H
#define COMMANDTASK_H

#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

void Command_Init(void);
bool Command_Parse(const char *cmd_str, uint16_t len);
const char *Command_GetLastStatus(void);
void Command_Task(void *pvParameters);

#endif
