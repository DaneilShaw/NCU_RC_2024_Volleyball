#include "AppRemote_Task.h"

Task_Flag task_flag;
int Cushion_Enable;
int Hit_Enable;
extern int Beta_Lock;
extern float Machine_Beta_Targetpos;

int flag;

void AppRemote_Task(void const * argument)
{
	uint32_t AppNotifyValue = pdFALSE;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		AppNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(AppNotifyValue == 1)
		{
/***************************************************************机械臂***************************************************************/
			if(AppRx_DataBuf[0] == 'B' && AppRx_DataBuf[1] == 'A' && AppRx_DataBuf[6] == 'L' && AppRx_DataBuf[7] == 'L')
			{
				/*通过APP发送指令后，标志位变为 txarm_ing*/
				task_flag.appdata_txarm_flag = txarm_ing;
				/*通过DMA发送 AppRx_DataBuf*/
				HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
				/*底盘设置为自动模式*/
				Chassis_Mode = Automatic_Mode;
				/*在完成传输之后，标志位会赋值为txarm_finish，直到标志位变为txarm_finish，才会进行下一步判断*/
				while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '0')
				{
					/*垫球击球标志位为0，垫球击球都不进行*/
					Cushion_Enable = 0;
					Hit_Enable = 0;
				}
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '1')
				{
					/*垫球标志位赋值为1 */
					Cushion_Enable = 1;
					Hit_Enable = 0;
				}
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '2')
				{
					/*击球标志位赋值为1 */
					Cushion_Enable = 0;
					Hit_Enable = 1;
				}
			}
/***************************************************************底盘***************************************************************/
			if(AppRx_DataBuf[0] == 'V' && AppRx_DataBuf[1] == 'S' && AppRx_DataBuf[6] == 'N' && AppRx_DataBuf[7] == 'T')
			{
				/*************能够实现用APP控制车的底盘运动，手动控制模式*************/
				Chassis_Mode = Operation_Mode;
				
				/*给一个恒定的速度,航向角根据app输出确定*/
				if(AppRx_DataBuf[3] != '4' && AppRx_DataBuf[3] != '9')
				{
					Chassis_World.Alpha = atoi((const char *)&AppRx_DataBuf[3]);
					Chassis_World.Vel = 2000.0f;
					/*	(const char *)&AppRx_DataBuf[3]:它将AppRx_DataBuf[3]的地址转换为一个指向const char的指针*/
					/*	atoi():它的作用是将一个字符串（const char*）转换成整型（int）*/
					/*	从AppRx_DataBuf数组的第四个字节开始，将接下来的字节序列解释为一个字符串，该字符串表示一个整数值*/
				}
				if(AppRx_DataBuf[3] == '9'&& AppRx_DataBuf[4] == '9' && AppRx_DataBuf[5] == '9')
				{
					Chassis_World.Alpha = 0;
					Chassis_World.Vel = 0;
				}
				
				/*给定角速度自转*/
				if(AppRx_DataBuf[3] == '4' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '1' )
			  {
					Chassis_Machine.velW = -1.0f;
					Beta_Lock = 0;
			  }
			  if(AppRx_DataBuf[3] == '4' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '2' )
			  {
					Chassis_Machine.velW = 1.0f;
					Beta_Lock = 0;
			  }
				if(AppRx_DataBuf[3] == '4' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '3' )
			  {
					Chassis_Machine.velW = 0.0f;
					Machine_Beta_Targetpos = Chassis_Machine.Beta;
					Beta_Lock = 1;
			  }
			}
		  memset(AppRx_DataBuf, 0, 8);
		}
		osDelayUntil(&xLastWakeTime, 3);//绝对延时
	}
}

