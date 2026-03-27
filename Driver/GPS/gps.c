#include "gps.h"
#include "stm32f4xx.h"
#include <string.h>

#define GPS_USART USART3
#define GPS_USART_IRQn USART3_IRQn
#define GPS_USART_CLK RCC_APB1Periph_USART3
#define GPS_GPIO_CLK RCC_AHB1Periph_GPIOB
#define GPS_TX_PIN GPIO_Pin_10
#define GPS_RX_PIN GPIO_Pin_11
#define GPS_GPIO_PORT GPIOB
#define GPS_TX_PINSOURCE GPIO_PinSource10
#define GPS_RX_PINSOURCE GPIO_PinSource11
#define GPS_AF GPIO_AF_USART3
#define GPS_BAUDRATE 38400

#define GPS_RX_BUF_SIZE 256
#define GPS_FRAME_BUF_SIZE 100

static char gps_rx_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t gps_rx_index = 0;

static char gps_frame_buf[GPS_FRAME_BUF_SIZE];
static volatile bool gps_frame_ready = false;

static void (*s_rx_callback)(void) = NULL;

static uint8_t GPS_GetCommaPos(const char *buf, uint8_t cx);
static uint32_t GPS_Pow(uint8_t m, uint8_t n);
static int GPS_Str2Num(const char *buf, uint8_t *dx);
static void GPS_UTCToBeijing(gps_time_t *utc, gps_time_t *beijing);

void GPS_Init(void (*rx_callback)(void)) {
  s_rx_callback = rx_callback;

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(GPS_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(GPS_USART_CLK, ENABLE);

  GPIO_PinAFConfig(GPS_GPIO_PORT, GPS_TX_PINSOURCE, GPS_AF);
  GPIO_PinAFConfig(GPS_GPIO_PORT, GPS_RX_PINSOURCE, GPS_AF);

  GPIO_InitStructure.GPIO_Pin = GPS_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPS_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPS_RX_PIN;
  GPIO_Init(GPS_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = GPS_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(GPS_USART, &USART_InitStructure);

  USART_ITConfig(GPS_USART, USART_IT_RXNE, ENABLE);
  USART_Cmd(GPS_USART, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = GPS_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  gps_rx_index = 0;
  gps_frame_ready = false;
}

void USART3_IRQHandler(void) {
  if (USART_GetITStatus(GPS_USART, USART_IT_RXNE) != RESET) {
    char ch = USART_ReceiveData(GPS_USART);

    if (ch == '$') {
      gps_rx_index = 0;
    }

    if (gps_rx_index < GPS_RX_BUF_SIZE - 1) {
      gps_rx_buf[gps_rx_index++] = ch;
    }

    if (gps_rx_index >= 6 && gps_rx_buf[0] == '$' && 
        (gps_rx_buf[4] == 'M' && gps_rx_buf[5] == 'C')) {
      if (ch == '\n') {
        int copy_len = gps_rx_index;
        if(copy_len > GPS_FRAME_BUF_SIZE - 1) copy_len = GPS_FRAME_BUF_SIZE - 1;
        memcpy(gps_frame_buf, gps_rx_buf, copy_len);
        gps_frame_buf[copy_len] = '\0';
        gps_frame_ready = true;
        
        gps_rx_index = 0;

        if (s_rx_callback != NULL) {
            s_rx_callback();
        }
      }
    }

    if (gps_rx_index >= GPS_RX_BUF_SIZE) {
      gps_rx_index = GPS_RX_BUF_SIZE - 1;
    }
  }
}

bool GPS_GetRawFrame(char *buf, uint16_t max_len) {
    if(!gps_frame_ready) return false;
    
    strncpy(buf, gps_frame_buf, max_len - 1);
    buf[max_len - 1] = '\0';
    gps_frame_ready = false;
    
    return true;
}

bool GPS_ParseNMEA(const char *frame_buf, gps_data_t *parsed_data) {
  uint8_t posx, dx;
  uint32_t temp;
  float rs;
  const char *p;
  gps_time_t utc_time = {0};

  p = strstr(frame_buf, "$GNRMC");
  if (p == NULL) {
    p = strstr(frame_buf, "$GPRMC");
  }
  if (p == NULL) {
    return false;
  }

  /* 1：UTC时间 hhmmss.sss */
  posx = GPS_GetCommaPos(p, 1);
  if (posx != 0xFF) {
    temp = GPS_Str2Num(p + posx, &dx) / GPS_Pow(10, dx);
    utc_time.hour = temp / 10000;
    utc_time.min = (temp / 100) % 100;
    utc_time.sec = temp % 100;
  } else return false;

  /* 2：定位状态 A=有效, V=无效 */
  posx = GPS_GetCommaPos(p, 2);
  if (posx != 0xFF) {
    parsed_data->is_valid = (*(p + posx) == 'A');
  } else parsed_data->is_valid = false;

  /* 3,4：纬度 ddmm.mmmm N/S */
  posx = GPS_GetCommaPos(p, 3);
  if (posx != 0xFF) {
    temp = GPS_Str2Num(p + posx, &dx);
    float lat_deg = temp / GPS_Pow(10, dx + 2);
    rs = temp % GPS_Pow(10, dx + 2);
    parsed_data->latitude = lat_deg + (rs / GPS_Pow(10, dx)) / 60.0f;
  }
  posx = GPS_GetCommaPos(p, 4);
  if (posx != 0xFF && *(p + posx) == 'S') {
    parsed_data->latitude = -parsed_data->latitude;
  }

  /* 5,6：经度 dddmm.mmmm E/W */
  posx = GPS_GetCommaPos(p, 5);
  if (posx != 0xFF) {
    temp = GPS_Str2Num(p + posx, &dx);
    float lon_deg = temp / GPS_Pow(10, dx + 2);
    rs = temp % GPS_Pow(10, dx + 2);
    parsed_data->longitude = lon_deg + (rs / GPS_Pow(10, dx)) / 60.0f;
  }
  posx = GPS_GetCommaPos(p, 6);
  if (posx != 0xFF && *(p + posx) == 'W') {
    parsed_data->longitude = -parsed_data->longitude;
  }

  /* 9：UTC日期 ddmmyy */
  posx = GPS_GetCommaPos(p, 9);
  if (posx != 0xFF) {
    temp = GPS_Str2Num(p + posx, &dx);
    utc_time.day = temp / 10000;
    utc_time.month = (temp / 100) % 100;
    utc_time.year = 2000 + temp % 100;
  }

  /* UTC转北京时间 */
  GPS_UTCToBeijing(&utc_time, &parsed_data->time);
  return true;
}

static uint8_t GPS_GetCommaPos(const char *buf, uint8_t cx) {
  const char *p = buf;
  while (cx) {
    if (*buf == '*' || *buf < ' ' || *buf > 'z') {
      return 0xFF; 
    }
    if (*buf == ',') cx--;
    buf++;
  }
  return (uint8_t)(buf - p);
}

static uint32_t GPS_Pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--) result *= m;
  return result;
}

static int GPS_Str2Num(const char *buf, uint8_t *dx) {
  const char *p = buf;
  uint32_t ires = 0, fres = 0;
  uint8_t ilen = 0, flen = 0, i;
  uint8_t mask = 0;
  int res;

  while (1) {
    if (*p == '-') { mask |= 0x02; p++; }
    if (*p == ',' || *p == '*') break;
    if (*p == '.') { mask |= 0x01; p++; }
    else if (*p > '9' || *p < '0') { ilen = 0; flen = 0; break; }
    if (mask & 0x01) flen++;
    else ilen++;
    p++;
  }
  if (mask & 0x02) buf++;

  for (i = 0; i < ilen; i++) {
    ires += GPS_Pow(10, ilen - 1 - i) * (buf[i] - '0');
  }
  if (flen > 5) flen = 5;
  *dx = flen;
  for (i = 0; i < flen; i++) {
    fres += GPS_Pow(10, flen - 1 - i) * (buf[ilen + 1 + i] - '0');
  }

  res = ires * GPS_Pow(10, flen) + fres;
  if (mask & 0x02) res = -res;

  return res;
}

static void GPS_UTCToBeijing(gps_time_t *utc, gps_time_t *beijing) {
  uint16_t year = utc->year;
  uint8_t month = utc->month;
  uint8_t day = utc->day;
  uint8_t hour = utc->hour + 8; 

  uint8_t days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    days_in_month[2] = 29;
  }

  if (hour >= 24) {
    hour -= 24;
    day++;
    if (day > days_in_month[month]) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }

  beijing->year = year;
  beijing->month = month;
  beijing->day = day;
  beijing->hour = hour;
  beijing->min = utc->min;
  beijing->sec = utc->sec;
}
