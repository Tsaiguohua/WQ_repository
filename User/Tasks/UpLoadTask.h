#ifndef UPLOADTASK_H
#define UPLOADTASK_H

#include <stdbool.h>
#include <stdint.h>

void Upload_Task(void *pvParameters);
uint16_t Upload_GetFrequency(void);
bool Upload_SetFrequency(uint16_t freq_seconds);
void Upload_SendNow(void);

#endif
