#ifndef __SCOPE_PRINTF_H
#define __SCOPE_PRINTF_H
#include "usart.h"
#include "bsp_usart.h"
#include "pid.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define pw_uart huart8
#define dma_pw_rx hdma_uart8_rx

//extern uint8_t name[];//示波器发送数据名称
extern float channels[7];//示波器发送数据值


extern void usart_printf(const char *fmt,...);
extern void send_wave(uint8_t *name, uint8_t name_len, float *channels_t, uint16_t channel_len); 

#endif

