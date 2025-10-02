#include "GIM4310_steadywin.h"
#include "main.h"
#include "Motor_Feedback.h"

CAN_TxHeaderTypeDef  steady_tx_message;
uint8_t              steady_can_send_data[8];
float                Steadymotor_Basepos;
//uint16_t             steady_motorcnt; 

/*****************************�ֲ�����*****************************/


/**************************************************************
	@brief:		CANͨ�Ų�����ʼ��
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void MXReqInit(void)
{
	steady_tx_message.StdId = 0x01;
	steady_tx_message.IDE = CAN_ID_STD;
	steady_tx_message.RTR = CAN_RTR_DATA;
	steady_tx_message.DLC = 0x08;
}
/********************************end*******************************/
/*****************************ȫ�ֺ���*****************************/


/**************************************************************
	@brief:		GIM4310-10�������
	@param:		
	@retval: 		
	@supplement:	�Ե�����п���ǰ�������������
**************************************************************/
void MXReqStart(void)
{
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x91;
	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}
/**************************************************************
	@brief:		GIM4310-10���ֹͣ
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void MXReqStop(void)
{
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x92;
	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}

/**************************************************************
	@brief:		GIM4310-10����������ؿ���ģʽ
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void MXReqToqueControl(float toque,uint32_t duration)
{
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x93;
	uint8_t * toquePtr = (uint8_t *)(&toque);
	uint8_t * durationPtr = (uint8_t *)(&duration);
	
	steady_can_send_data[1] = toquePtr[0];
	steady_can_send_data[2] = toquePtr[1];
	steady_can_send_data[3] = toquePtr[2];
	steady_can_send_data[4] = toquePtr[3];
	
	steady_can_send_data[5] = durationPtr[0];
	steady_can_send_data[6] = durationPtr[1];
	steady_can_send_data[7] = durationPtr[2];

	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}

/**************************************************************
	@brief:		GIM4310-10��������ٶȿ���ģʽ
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void MXReqSpeedControl(float speed,uint32_t duration)
{
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x94;
	uint8_t * speedPtr = (uint8_t *)(&speed);
	uint8_t * durationPtr = (uint8_t *)(&duration);
	
	steady_can_send_data[1] = speedPtr[0];
	steady_can_send_data[2] = speedPtr[1];
	steady_can_send_data[3] = speedPtr[2];
	steady_can_send_data[4] = speedPtr[3];
	
	steady_can_send_data[5] = durationPtr[0];
	steady_can_send_data[6] = durationPtr[1];
	steady_can_send_data[7] = durationPtr[2];

	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}

/**************************************************************
	@brief:		���״̬��ѯ
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
//void MXReqInquire(uint8_t IndID)
//{
//	uint32_t send_mail_box;
//	MXReqInit();
//	steady_can_send_data[0] = 0xB4;
//	steady_can_send_data[1] = IndID;
//	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
//}

/**************************************************************
	@brief:		GIM4310-10�������λ�ÿ���ģʽ
	@param:		increment_flag����0���þ���λ��ģʽ��
                            ��1�������λ��ģʽ
	@retval: 		
	@supplement:	
**************************************************************/
 void MCReqPositionControl(float position,uint32_t duration)
{
	static uint8_t increment_flag = 1;//ʹ�þ�̬�ֲ�����
  if(increment_flag)
	{
	  MXReqSpeedControl(0.0f, 5);
	  HAL_Delay(2);
	  Steadymotor_Basepos = Steady_Rx_Data.position;//��ȡ�����λ��ֵ
		increment_flag = 0;
	}
	position = Steadymotor_Basepos + position;
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x95;
	uint8_t * positionPtr = (uint8_t *)(&position);
	uint8_t * durationPtr = (uint8_t *)(&duration);
	
	steady_can_send_data[1] = positionPtr[0];
	steady_can_send_data[2] = positionPtr[1];
	steady_can_send_data[3] = positionPtr[2];
	steady_can_send_data[4] = positionPtr[3];
	
	steady_can_send_data[5] = durationPtr[0];
	steady_can_send_data[6] = durationPtr[1];
	steady_can_send_data[7] = durationPtr[2];

	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}

/**************************************************************
	@brief:		�ж�ָ�����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void MXReqCancel(void)
{
	uint32_t send_mail_box;
	MXReqInit();
	steady_can_send_data[0] = 0x97;
	HAL_CAN_AddTxMessage(&hcan1, &steady_tx_message, steady_can_send_data, &send_mail_box);
}
/********************************END*******************************/

