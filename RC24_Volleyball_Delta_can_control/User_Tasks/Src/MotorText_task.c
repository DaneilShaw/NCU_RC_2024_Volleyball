#include "MotorText_task.h"
#include "Scope_printf.h"

float channels[7];
uint8_t len;
extern motor_state_s HTM_StateData;
extern int time;
//void MotorTextTask(void const * argument)
//{

///************************************���ڴ�ӡ�������ԣ����ֻ�ǲ��ԣ�ʵ�������������ʹ�ã�************************************/
////	int integer_variable = 100;
////	char string_variable[] = "abc";
/////************************************ʾ������������************************************/
////	char name[] = "text1,text2,text3,text4,text5,text6,text7";
////  channels[0] = 10.0f;
//// 	channels[1] = 20.0f;
//// 	channels[2] = 30.0f;
//// 	channels[3] = 40.0f;
//// 	channels[4] = 50.0f;
//// 	channels[5] = 60.0f;
////	channels[6] = 70.0f;
/////************************************�󽮵��������������************************************/
////	pid_init(&GM6020_PIDpos, Position_Mode, 1.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
////	pid_init(&GM6020_PIDspd, Position_Mode, 1.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
////	GM6020_State_Data.targetspd = 2000;
/////************************************HTM5046���������������************************************/
////	HTMotor_Write(&HTM_StateData, 1.5f, 3.0f, 5000);
////  motor_control_pos_val_tqe(0x01, &HTM_StateData);//��λ�ٻ�Ͽ���
////	motor_read(0x01);//���ݶ�ȡ
/////************************************ ���Ժ�������End************************************/


//char name[] = "GO_TailArm_RecvData.Pos,GO_TailArm_RecvData.T,GO_TailArm_RecvData.W";
//////  char name[] = "TwoArm_Target,ThreeArm_Target,FourArm_Target,TwoArm_W,ThreeArm_W,FourArm_W";
//portTickType xLastWakeTime;
//xLastWakeTime = xTaskGetTickCount();
////	
//for(;;)
//{	
//	channels[0] = GO_TailArm_RecvData.Pos;
//	channels[1] = GO_TailArm_RecvData.T;
//	channels[2] = GO_TailArm_RecvData.W;
////		channels[3] = GO_TwoArm_RecvData.Pos;
////		channels[4] = GO_ThreeArm_RecvData.Pos;
////		channels[5] = HTM_StateData.pos_rad;
//	
////		channels[3] = GO_TwoArm_RecvData.W;
////		channels[4] = GO_ThreeArm_RecvData.W;
////		channels[5] = HTM_StateData.angular_vel;
////		
////		usart_printf("���̽�������: %d \r\n", Transfer_RxData);//���ڴ�ӡ����
////		
//	send_wave((uint8_t *)name, sizeof(name), channels, sizeof(channels));//ʾ��������
//	
//	osDelayUntil(&xLastWakeTime,5);
//}
//}


