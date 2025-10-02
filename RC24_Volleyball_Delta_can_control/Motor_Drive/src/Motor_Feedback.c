#include "Motor_Feedback.h"

uint8_t Can1_RxData[8];//can1������������
uint8_t Can2_RxData[8];//can2������������
//steady���
Motctrl_Rx Steady_Rx_Data;
uint8_t RX_ID;
//unitree GO���
uint8_t GOM8010_Rx[GOM8010_RX_SIZE];
uint32_t GO_RxLen;
MOTOR_recv GO_TwoArm_RecvData;
MOTOR_recv GO_ThreeArm_RecvData;
MOTOR_recv GO_TailArm_RecvData;
//GM6020���
DjiMotor_State GM6020_State_Data;
//M3508���
DjiMotor_State M3508_State_Data[4];
//HTM_5046���
motor_state_s HTM_StateData;

/***************************���ݽ��պ�������****************************/
void HTM_RxDecode(motor_state_s * ptr, uint8_t * data, CAN_RxHeaderTypeDef * CAN_Header);
void M3508_RxDecode(DjiMotor_State * ptr, uint8_t * data);
/***********************************************************************/


/**************************************************************
	@brief:		CAN�жϻص�����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Can1_Rx_Header;
	CAN_RxHeaderTypeDef Can2_Rx_Header;
	if(hcan == &hcan1)//1M������
	{
	  /***************��ȡ��������***************/
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can1_Rx_Header, Can1_RxData);
		/***************id�ж�***************/
		switch(Can1_Rx_Header.StdId)
		{
/**********************************����**********************************/
			case 0x201:
		  case 0x202:
			case 0x203:
			case 0x204:
			{
				int id =  Can1_Rx_Header.StdId -0x201;
				M3508_RxDecode(&M3508_State_Data[id], Can1_RxData);
			} break;
/**********************************��е��**********************************/
			case 0x100:
			{
				HTM_RxDecode(&HTM_StateData, Can1_RxData, &Can1_Rx_Header);
			} break;
			
			case 0x209://GM6020�������̨�����0x209
			{
				GM6020_State_Data.pos = (uint16_t)((Can1_RxData[0] << 8) | (Can1_RxData[1]));
				GM6020_State_Data.spd = (int16_t)(Can1_RxData[2] << 8) | (Can1_RxData[3]);
				GM6020_State_Data.current = (int16_t)(Can1_RxData[4] << 8) | (Can1_RxData[5]);
				GM6020_State_Data.temp = Can1_RxData[6];
				Get_Nowpos(&GM6020_State_Data);
			} break;
			
			default: break;
		}
	}
	if(hcan == &hcan2)//500K������
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can2_Rx_Header, Can2_RxData);
		int cmd = Can2_Rx_Header.ExtId >> 8;
		int i = (Can2_Rx_Header.ExtId & 0xFF) -1;
		switch(cmd)
		{
			case CAN_PACKET_STATUS://�����������
			{
				can_vescmsg[i].id = Can2_Rx_Header.ExtId & 0xFF;
				can_vescmsg[i].rx_time = 0;  
				can_vescmsg[i].rpm = Can2_RxData[0] << 24 | Can2_RxData[1] << 16 | Can2_RxData[2] << 8 | Can2_RxData[3];
				can_vescmsg[i].rpm = can_vescmsg[i].rpm /poles;
				can_vescmsg[i].current = (float)(Can2_RxData[4] << 8 | Can2_RxData[5]) / 10.0f;
				can_vescmsg[i].duty = (float)(Can2_RxData[6] << 8 | Can2_RxData[7]) / 1000.0f;
			} break;
			
			default: break;
		}
	}
}

/**************************************************************
	@brief:		����6�����жϻص�����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
int go_flag = 0;
//uint32_t a1_time_last, a1_time;//��ѯʱ��������
void UnitreeGo_USART6_IRQHandler(void)
{
	//�ж��Ƿ�Ϊ�����ж�
	if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
	{
		go_flag++;
		BaseType_t pxHigherPriorityTaskWoken;
		
		uint8_t id;
		/*��������жϱ�־*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
		/*��ȡ���յ���GO������ݳ����뷢�͵��Ƿ�һ��*/
		GO_RxLen = DMA_USART6_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
		if(GO_RxLen == 0) 
		{
			GO_TailArm_RecvData.correct = -1;
			GO_ThreeArm_RecvData.correct = -1;
		}
		if(GO_RxLen == 16)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(GOM8010_Rx, DMA_USART6_RxBuf, 16);//�ӻ�������ȡ��������
			
			id = GOM8010_Rx[2] & 0x0F; //��ȡ���id
			
			switch(id)
			{
				case 0x01://3�ؽ�Go���
				{			
//					a1_time = HAL_GetTick() - a1_time_last;//�����ж���ѯʱ��
//          a1_time_last = HAL_GetTick();//�����ж���ѯʱ��
					GO_ThreeArm_RecvData.correct = 1;
				  memcpy(GO_ThreeArm_RecvData.motor_recv_data.head, GOM8010_Rx, 2); //��GOM8010_Rx��ǰ�����ֽڸ��Ƶ�head������
				  GO_ThreeArm_RecvData.motor_recv_data.mode.id = 1;
				  GO_ThreeArm_RecvData.motor_recv_data.mode.status = (GOM8010_Rx[2] >> 4) & 0x07;  // �� GOM8010_Rx[2] �еĸ� 3 λ��ֵ�� status
				  memcpy(&GO_ThreeArm_RecvData.motor_recv_data.fbk, GOM8010_Rx + 3, sizeof(RIS_Fbk_t)); //����RIS_Fbk_t���͵�����ռ����11���ֽڣ���GOM8010_Rx�ĵ��ĸ��ֽڿ�ʼ��11���ֽڸ��Ƶ�fbk��
				  GO_ThreeArm_RecvData.motor_recv_data.CRC16 = *((uint16_t*)(GOM8010_Rx + 14)); // ����CRC16ռ������������ֽڣ���GOM8010_Rx�ĵ����ڶ���������һ���ֽڹ��ɵ�uint16_t���ݸ�ֵ��CRC16
				  extract_data(&GO_ThreeArm_RecvData);//���ݽ���
				} break;
				case 0x00://2�ؽ�Go���
				{
					GO_TailArm_RecvData.correct = 1;
				  memcpy(GO_TailArm_RecvData.motor_recv_data.head, GOM8010_Rx, 2); //��GOM8010_Rx��ǰ�����ֽڸ��Ƶ�head������
				  GO_TailArm_RecvData.motor_recv_data.mode.id = 0;
				  GO_TailArm_RecvData.motor_recv_data.mode.status = (GOM8010_Rx[2] >> 4) & 0x07;  // �� GOM8010_Rx[2] �еĸ� 3 λ��ֵ�� status
				  memcpy(&GO_TailArm_RecvData.motor_recv_data.fbk, GOM8010_Rx + 3, sizeof(RIS_Fbk_t)); //����RIS_Fbk_t���͵�����ռ����11���ֽڣ���GOM8010_Rx�ĵ��ĸ��ֽڿ�ʼ��11���ֽڸ��Ƶ�fbk��
				  GO_TailArm_RecvData.motor_recv_data.CRC16 = *((uint16_t*)(GOM8010_Rx + 14)); // ����CRC16ռ������������ֽڣ���GOM8010_Rx�ĵ����ڶ���������һ���ֽڹ��ɵ�uint16_t���ݸ�ֵ��CRC16
				  extract_data(&GO_TailArm_RecvData);//���ݽ���
				} break;
				
				default: break;
			}
		}
		/*��������������*/
		memset(DMA_USART6_RxBuf, 0, DMA_USART6_RX_SIZE);
		/*�����趨���������ݳ���*/
		hdma_usart6_rx.Instance->NDTR = DMA_USART6_RX_SIZE;
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&hdma_usart6_rx);
		/*���Ӧ�����������֪ͨ*/
    vTaskNotifyGiveFromISR(ArmDrive_Handle, &pxHigherPriorityTaskWoken);
		/*�����Ҫ�Ļ�������һ�������л���ϵͳ�ж��Ƿ���Ҫ�����л�*/
	  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/***************************���ݽ��ս���****************************/
/**************************************************************
	@brief:		HTM����������ݽ��뺯��
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void HTM_RxDecode(motor_state_s * ptr, uint8_t * data, CAN_RxHeaderTypeDef * CAN_Header)
{
	ptr -> id = CAN_Header -> StdId;
	ptr -> position = data[2] | (data[3] << 8);
	ptr -> pos_rad = 2 * pi * 0.0001f * (ptr -> position);
	ptr -> velocity = data[4] | (data[5] << 8);
	ptr -> angular_vel = (ptr -> velocity) * 0.00025f * 2 * pi;
//	ptr -> velocity = 0.00025 * (ptr -> velocity);
	ptr -> torque = data[6] | (data[7] << 8);
//	ptr -> torque = 0.0001 * (ptr -> torque);
}

/**************************************************************
	@brief:		GIM4310-10����������ݽ��뺯��
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void Steady_RxDecode(Motctrl_Rx * ptr, uint8_t * data)
{
	ptr -> motctrl_cmd = data[0];
	
	if((data[0] == 0x91)||(data[0] == 0x92)||(data[0] == 0x93)||(data[0] == 0x94)|(data[0] == 0x95)||(data[0] == 0x97))
	{
		ptr -> res = data[1];
		ptr -> temperature = data[2];
		ptr -> position = ((data[4] << 8) | data[3]);
		ptr -> position = (ptr -> position) * 25 / 65535 - 12.5f;
		ptr -> speed = (data[5] << 4) | (data[6] >>4);
		ptr -> speed = ((ptr -> speed) * 130 / 4095 - 65) * 30 / pi;
		ptr -> toque = ((data[6] & 0x0F) << 4) | data[7];
		ptr -> toque = (ptr -> toque) * 1372.14f / 4095 - 686.07f; 
	}
	if(data[0] == 0xB4)
	{
	  ptr -> IndID = data[1];
		ptr -> res = data[2];
		switch(data[1])
		{
			//��ѯ����¶�
		  case MOTCTRL_INDIID_TEMP_MOTOR:
			{
				ptr -> temperature = data[7]<<24 | data[6]<<16 | data[5]<<8 | data[4];
			} break;
			//��ѯ������е��
			case MOTCTRL_INDIID_MEC_ANGLE_SHAFT:
			{
				const uint32_t posvalue = ((data[7]<<24) | (data[6]<<16) | (data[5]<<8) | (data[4]));
				float nowpos = *((float*)&posvalue);
				ptr -> position = nowpos / 36;
			} break;	
			default: break;
		}
	}
}
/**************************************************************
	@brief:		3508����������ݽ��뺯��
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void M3508_RxDecode(DjiMotor_State * ptr, uint8_t * data)
{
	ptr->pos = (uint16_t)((data[0] << 8) | (data[1]));
	ptr->spd = (int16_t)(data[2] << 8) | (data[3]);
	ptr->current = (int16_t)(data[4] << 8) | (data[5]);
	ptr->temp = data[6];
	Get_Nowpos(ptr);
}

