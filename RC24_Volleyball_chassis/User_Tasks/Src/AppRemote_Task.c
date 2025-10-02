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
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	for(;;)
	{
		AppNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(AppNotifyValue == 1)
		{
/***************************************************************��е��***************************************************************/
			if(AppRx_DataBuf[0] == 'B' && AppRx_DataBuf[1] == 'A' && AppRx_DataBuf[6] == 'L' && AppRx_DataBuf[7] == 'L')
			{
				/*ͨ��APP����ָ��󣬱�־λ��Ϊ txarm_ing*/
				task_flag.appdata_txarm_flag = txarm_ing;
				/*ͨ��DMA���� AppRx_DataBuf*/
				HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
				/*��������Ϊ�Զ�ģʽ*/
				Chassis_Mode = Automatic_Mode;
				/*����ɴ���֮�󣬱�־λ�ḳֵΪtxarm_finish��ֱ����־λ��Ϊtxarm_finish���Ż������һ���ж�*/
				while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '0')
				{
					/*��������־λΪ0��������򶼲�����*/
					Cushion_Enable = 0;
					Hit_Enable = 0;
				}
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '1')
				{
					/*�����־λ��ֵΪ1 */
					Cushion_Enable = 1;
					Hit_Enable = 0;
				}
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '2')
				{
					/*�����־λ��ֵΪ1 */
					Cushion_Enable = 0;
					Hit_Enable = 1;
				}
			}
/***************************************************************����***************************************************************/
			if(AppRx_DataBuf[0] == 'V' && AppRx_DataBuf[1] == 'S' && AppRx_DataBuf[6] == 'N' && AppRx_DataBuf[7] == 'T')
			{
				/*************�ܹ�ʵ����APP���Ƴ��ĵ����˶����ֶ�����ģʽ*************/
				Chassis_Mode = Operation_Mode;
				
				/*��һ���㶨���ٶ�,����Ǹ���app���ȷ��*/
				if(AppRx_DataBuf[3] != '4' && AppRx_DataBuf[3] != '9')
				{
					Chassis_World.Alpha = atoi((const char *)&AppRx_DataBuf[3]);
					Chassis_World.Vel = 2000.0f;
					/*	(const char *)&AppRx_DataBuf[3]:����AppRx_DataBuf[3]�ĵ�ַת��Ϊһ��ָ��const char��ָ��*/
					/*	atoi():���������ǽ�һ���ַ�����const char*��ת�������ͣ�int��*/
					/*	��AppRx_DataBuf����ĵ��ĸ��ֽڿ�ʼ�������������ֽ����н���Ϊһ���ַ��������ַ�����ʾһ������ֵ*/
				}
				if(AppRx_DataBuf[3] == '9'&& AppRx_DataBuf[4] == '9' && AppRx_DataBuf[5] == '9')
				{
					Chassis_World.Alpha = 0;
					Chassis_World.Vel = 0;
				}
				
				/*�������ٶ���ת*/
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
		osDelayUntil(&xLastWakeTime, 3);//������ʱ
	}
}

