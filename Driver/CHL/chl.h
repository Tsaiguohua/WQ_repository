#ifndef CHL_H
#define CHL_H

#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////
// CHL（叶绿素）传感器驱动
//
// Modbus地址：0x03
//////////////////////////////////////////////////////////////////////////////////

bool chl_open(void);
bool chl_close(void);
bool chl_read(float *chl_value);
bool chl_open_brush(void);
bool chl_get_brush_time(uint16_t *interval);
bool chl_set_brush_time(uint16_t interval);

#endif
