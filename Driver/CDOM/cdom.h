#ifndef CDOM_H
#define CDOM_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// CDOM（有色溶解有机物）传感器驱动
//
// Modbus地址：0x02
//////////////////////////////////////////////////////////////////////////////////

bool cdom_open(void);
bool cdom_close(void);
bool cdom_read(float *cdom_value);
bool cdom_open_brush(void);
bool cdom_get_brush_time(uint16_t *interval);
bool cdom_set_brush_time(uint16_t interval);

#endif
