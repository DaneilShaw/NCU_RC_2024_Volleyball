#ifndef MOTOR_BACK_H
#define MOTOR_BACK_H
#include "My_Includes.h"
#include "GIM4310_steadywin.h"
#include "GO_M8010_6.h"
#include "DjiMotor.h"
#include "vesc6.h"
#include "HTM_5046.h"
#include "math_formula.h"
#include "Filter_Algorithm.h"

#define GOM8010_RX_SIZE 16

extern uint8_t Can1_RxData[8];
extern Motctrl_Rx Steady_Rx_Data;
extern DjiMotor_State GM6020_State_Data;
extern MOTOR_recv GO_TwoArm_RecvData;
extern MOTOR_recv GO_ThreeArm_RecvData;
extern DjiMotor_State M3508_State_Data[3];

extern void UnitreeGo_USART6_IRQHandler(void);

#endif

