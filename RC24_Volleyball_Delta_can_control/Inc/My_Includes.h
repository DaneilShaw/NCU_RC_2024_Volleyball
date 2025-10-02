#ifndef __MY_INCLUDES_H
#define __MY_INCLUDES_H

/***************************************�ײ�Դ�ļ�***************************************/

#include "gpio.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

/***************************************���γ�ʼ��Դ�ļ�***************************************/

#include "bsp.h"

/***************************************C����Դ�ļ�***************************************/

#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "struct_typedef.h"

/***************************************����ϵͳԴ�ļ�***************************************/

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

extern osThreadId AppRemote_Handle;
extern osThreadId ArmDrive_Handle;
extern osThreadId Vision_Handle;
extern osThreadId Serve_Handle;
extern osThreadId HitBall_Handle;

/***************************************����***************************************/

#endif




