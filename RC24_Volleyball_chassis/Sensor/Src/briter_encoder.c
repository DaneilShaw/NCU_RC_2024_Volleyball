#include "briter_encoder.h"

uint8_t Briter_TxData[8];

/**************************************************************
	@brief:		�����ؾ���ֵ���������ݷ��ͺ���
	@param:		
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
uint8_t Encoder_Set_Function(Encoder_Struct *Encoder_Structure)            //�����ؾ���ֵ����������
{
	uint32_t msgbox;
	CAN_TxHeaderTypeDef	Briter_Txmessage;

	Briter_Txmessage.StdId = Encoder_Structure->Encoder_ID;							             //������ID
	Briter_Txmessage.IDE		= Encoder_Structure->Data_Format;						             //��׼֡��ʽ CAN_ID_STD 0
	Briter_Txmessage.RTR		= Encoder_Structure->Data_Type;							             //����֡ CAN_RTR_DATA 0
	Briter_Txmessage.ExtId = 0;																		                   //��չID
	Briter_Txmessage.DLC 	= 8;				                       //���͵����ݳ���,�ֽ���

	Briter_TxData[0]	= (Encoder_Structure->Front_Data[0]);
	Briter_TxData[1]	= (Encoder_Structure->Front_Data[1]);
	Briter_TxData[2]	= (Encoder_Structure->Front_Data[2]);	
	Briter_TxData[3]	= (Encoder_Structure->Front_Data[3]);
	Briter_TxData[4]	= (Encoder_Structure->Front_Data[4]);
	Briter_TxData[5]	= (Encoder_Structure->Front_Data[5]);
	Briter_TxData[6]	= (Encoder_Structure->Front_Data[6]);
	Briter_TxData[7]	= (Encoder_Structure->Front_Data[7]);
/******���������Ϣ���Ӳ������������******/
	HAL_CAN_AddTxMessage(&hcan2, &Briter_Txmessage, Briter_TxData, &msgbox);
/***************��ѯ��Ϣ�������**************/
	while(HAL_CAN_AddTxMessage(&hcan2, &Briter_Txmessage, Briter_TxData, &msgbox)!=HAL_OK);

/******��Ϣ���ͳɹ�*****/
	return 0;
}

/**************************************************************
	@brief:		��������ǰ�ǶȲ�ѯ����
	@param:		
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
void encoder_read_angle(Encoder_Struct *control_Encoder, int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x01;
	control_Encoder->Front_Data[3] = 0x00;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
  Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		������ID���ú���
	@param:		
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
void encoder_set_id(Encoder_Struct *control_Encoder, int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = 0x01;//��������ʼIDΪ0x01,����ֵ�������ID�仯���仯
	control_Encoder->Front_Data[2] = 0x02;
	control_Encoder->Front_Data[3] = id;//Ҫ���õ�id
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		���������������ú���
	@param:		
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
void encoder_set_baud(Encoder_Struct *control_Encoder,int8_t id, int16_t baud)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x03;
	if(baud==500)control_Encoder->Front_Data[3] = 0x00;
	if(baud==1000)control_Encoder->Front_Data[3] = 0x01;
	if(baud==250)control_Encoder->Front_Data[3] = 0x02;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		����������ģʽ���ú���
	@param:		
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
void encoder_set_mode(Encoder_Struct *control_Encoder,int8_t id, int8_t mode)
{	
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x04;
	if(mode==reference_mode)control_Encoder->Front_Data[3] = 0x00;//��ѯģʽ
	if(mode==backhaul_mode)control_Encoder->Front_Data[3] = 0xAA;//�Զ��ط�ģʽ
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		�ش��ٶ����ú���
	@param:		�ٶ�̫�콫�޷���������������03E8Ϊ1000΢��
	@retval: 		
	@supplement:	�����ؾ���ֵ����������
**************************************************************/
void encoder_set_speed(Encoder_Struct *control_Encoder,int8_t id, uint16_t speed)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x05;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x05;
	control_Encoder->Front_Data[3] = speed && 0xff;//��ȡ��8λ
	control_Encoder->Front_Data[4] = speed >> 8;//��ȡ��8λ
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		������ú���
	@param:		
	@retval: 		
	@supplement:	�趨��ǰλ��ֵΪ���
**************************************************************/
void encoder_set_zero(Encoder_Struct *control_Encoder, int8_t id)//�趨��ǰλ��ֵΪ���
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x06;
	control_Encoder->Front_Data[3] = 0x00;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		�����������ú���
	@param:		
	@retval: 		
	@supplement:	�趨������������
**************************************************************/
void encoder_set_direction(Encoder_Struct *control_Encoder,int8_t id, uint16_t direction)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x07;
	if(direction==clockwise)
		control_Encoder->Front_Data[3] = 0x00;//������˳ʱ�����
	if(direction==anticlockwise)
		control_Encoder->Front_Data[3] = 0x01;//��������ʱ�����
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		�е����ú���
	@param:		
	@retval: 		
	@supplement:	�趨��ǰλ��Ϊ�е�
**************************************************************/
void encoder_set_half(Encoder_Struct *control_Encoder,int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x0C;
	control_Encoder->Front_Data[3] = 0x01;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}



