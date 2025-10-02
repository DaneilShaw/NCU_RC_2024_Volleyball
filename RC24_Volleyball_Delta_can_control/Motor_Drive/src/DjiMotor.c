#include "DjiMotor.h"
#include "Motor_Feedback.h"

CAN_TxHeaderTypeDef  GM6020_tx_message;
uint8_t GM6020_send_data[8];
uint8_t M3508_send_data[8];
//GM6020_State GM6020_State_Data;

/*****************************局部函数*****************************/
/********************************end*******************************/

/*****************************全局函数*****************************/
/**************************************************************
	@brief:		GM6020电机参数设置
	@param:		
	@retval: 		
	@supplement:	GM6020电机控制
**************************************************************/
void GM6020_Write(DjiMotor_State * ptr_State, float tgpos, float tgspd, float tgcurrent)
{
	ptr_State->targetpos = tgpos;
	ptr_State->targetspd = tgspd;
	ptr_State->targetcurrent = tgcurrent;
}

/**************************************************************
	@brief:		大疆电机即时位置值获取
	@param:		
	@retval: 		
	@supplement:	GM6020电机控制
**************************************************************/
void Get_Nowpos(DjiMotor_State * ptr_State)
{
	if(ptr_State->pos - ptr_State->lastpos < -4096)
		ptr_State->num++;
	else if(ptr_State->pos - ptr_State->lastpos > 4096)
		ptr_State->num--;
	ptr_State->lastpos = ptr_State->pos;
	ptr_State->sumpos = ptr_State->num * 8192 + ptr_State->pos - ptr_State->basepos;
}

/**************************************************************
	@brief:		GM6020电机初始位置值获取
	@param:		
	@retval: 		
	@supplement:	GM6020电机控制
**************************************************************/
void Get_Basepos(DjiMotor_State * ptr)
{
	ptr->num = 0;
	while(ptr->basepos == 0)
	{
		ptr->basepos = ptr->pos;
	}
	ptr->num = 0;//避免电机在未转动的情况下已计一圈
	ptr->sumpos = 0;
}

/**************************************************************
	@brief:		GM6020电机启动
	@param:		
	@retval: 		
	@supplement:	GM6020电机控制
**************************************************************/
void GM6020_CAN_cmd(uint32_t identifier, int16_t yaw)
{
	uint32_t send_mail_box;
	GM6020_tx_message.StdId = identifier;
	GM6020_tx_message.IDE = CAN_ID_STD;
	GM6020_tx_message.RTR = CAN_RTR_DATA;
	GM6020_tx_message.DLC = 0x08;
	
	GM6020_send_data[0] = yaw >> 8;
	GM6020_send_data[1] = yaw;
	GM6020_send_data[2] = 0x00;
	GM6020_send_data[3] = 0x00;
	GM6020_send_data[4] = 0x00;
	GM6020_send_data[5] = 0x00;
	GM6020_send_data[6] = 0x00;
	GM6020_send_data[7] = 0x00;
	
	HAL_CAN_AddTxMessage(&hcan1, &GM6020_tx_message, GM6020_send_data, &send_mail_box);
}

/**************************************************************
	@brief:		M3508电机启动
	@param:		
	@retval: 		
	@supplement:	M3508电机控制
**************************************************************/
void M3508_CAN_cmd(uint32_t identifier, int16_t wheel1, int16_t wheel2, int16_t wheel3, int16_t wheel4)
{
	uint32_t send_mail_box;
	GM6020_tx_message.StdId = identifier;
	GM6020_tx_message.IDE = CAN_ID_STD;
	GM6020_tx_message.RTR = CAN_RTR_DATA;
	GM6020_tx_message.DLC = 0x08;
	
	M3508_send_data[0] = wheel1 >> 8;
	M3508_send_data[1] = wheel1;
	M3508_send_data[2] = wheel2 >> 8;
	M3508_send_data[3] = wheel2;
	M3508_send_data[4] = wheel3 >> 8;
	M3508_send_data[5] = wheel3;
	M3508_send_data[6] = wheel4 >> 8;
	M3508_send_data[7] = wheel4;
	
	HAL_CAN_AddTxMessage(&hcan1, &GM6020_tx_message, M3508_send_data, &send_mail_box);
}


