#include "MotorText_task.h"
#include "Scope_printf.h"

float channels[6];
uint8_t len;
extern motor_state_s HTM_StateData;
extern float Kalman_X_axis;
extern float Kalman_Y_axis;
extern float Vision_Inertial_Xspd, Vision_Inertial_Yspd;

extern float Machine_Beta_Basepos, Machine_Beta_Targetpos;
extern Machine_Coordinate Chassis_Machine;
extern pid_t Machine_Beta_PIDpos;

extern int32_t now_time[2], last_time;


extern Camera_Coordinate_t Camera_Coordinate;

void MotorTextTask(void const * argument)
{
///************************************串口打印函数测试************************************/
//	int integer_variable = 100;
//	char string_variable[] = "abc";
///************************************示波器函数测试************************************/
//	char name[] = "text1,text2,text3,text4,text5,text6,text7";
//  channels[0] = 10.0f;
// 	channels[1] = 20.0f;
// 	channels[2] = 30.0f;
// 	channels[3] = 40.0f;
// 	channels[4] = 50.0f;
// 	channels[5] = 60.0f;
//	channels[6] = 70.0f;
///************************************大疆电机驱动函数测试************************************/
//	pid_init(&GM6020_PIDpos, Position_Mode, 1.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
//	pid_init(&GM6020_PIDspd, Position_Mode, 1.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
//	GM6020_State_Data.targetspd = 2000;
///************************************HTM5046电机驱动函数测试************************************/
//	HTMotor_Write(&HTM_StateData, 1.5f, 3.0f, 5000);
//  motor_control_pos_val_tqe(0x01, &HTM_StateData);//力位速混合控制
//	motor_read(0x01);//数据读取
/************************************End************************************/	
//  char name[] = "Camera_Coordinate.X_axis,Camera_Coordinate.Y_axis,Kalman_X_axis,Kalman_Y_axis";
	char name[] = "Target1,Target2,Target3,Speed1,Speed2,Speed3";

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
  {	
		vTaskDelay(10);
		
		channels[0] = Chassis_Wheel.wheel1;
		channels[1] = Chassis_Wheel.wheel2;
		channels[2] = Chassis_Wheel.wheel3;
		channels[3] = M3508_State_Data[0].spd;
		channels[4] = M3508_State_Data[1].spd;
		channels[5]=  M3508_State_Data[2].spd;
		
//		usart_printf("目标速度: %d 实际速度: %d\r\n", (int16_t)Chassis_Wheel.wheel1, M3508_State_Data[0].spd);//串口打印函数
//		
//		usart_printf("时间间隔: %d \r\n", now_time[1]);//串口打印函数
		
		send_wave((uint8_t *)name, sizeof(name), channels, sizeof(channels));//示波器函数
		
		osDelayUntil(&xLastWakeTime, 5);
  }
}


