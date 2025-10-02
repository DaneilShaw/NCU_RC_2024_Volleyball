#ifndef __MY_INCLUDES_H
#define __MY_INCLUDES_H

/***************************************底层源文件***************************************/

#include "gpio.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

/***************************************二次初始化源文件***************************************/

#include "bsp.h"

/***************************************C语言源文件***************************************/

#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "struct_typedef.h"

/***************************************操作系统源文件***************************************/

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

extern osThreadId AppRemote_Handle;
extern osThreadId ArmDrive_Handle;
extern osThreadId Vision_Handle;
extern osThreadId Serve_Handle;
extern osThreadId HitBall_Handle;

/***************************************结束***************************************/

#endif




