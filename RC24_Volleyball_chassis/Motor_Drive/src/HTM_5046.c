#include "HTM_5046.h"
#include "math_formula.h"

CAN_TxHeaderTypeDef  HTM5046_tx_message;
//uint8_t TxData[8] = {0};

/**************************************************************
	@brief:		HTM5046�����������
	@param:		
	@retval: 		
	@supplement:	HTM5046�������
**************************************************************/
void HTMotor_Write(motor_state_s * ptr_state, float val, float tqe, float pos)
{
	ptr_state->val = val;
	ptr_state->tqe = tqe;
	ptr_state->pos = pos;
}

/**************************************************************
	@brief:		HTM������ݷ���
	@param:		
	@retval: 		
	@supplement:	HTM5046�������
**************************************************************/
void CAN_Send_Msg(uint32_t id, uint8_t * msg, uint8_t len)
{
	uint32_t mailbox;
	uint8_t TxData[8] = {0};
	HTM5046_tx_message.ExtId = id;                  //������չ֡ID
	HTM5046_tx_message.IDE = CAN_ID_EXT;            //ʹ����չID
	HTM5046_tx_message.RTR = CAN_RTR_DATA;          //����֡
	HTM5046_tx_message.DLC = 0x08;                  //���ݳ���
	
	for(int i = 0; i < len; i++)
	{
		TxData[i] = msg[i];
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &HTM5046_tx_message, TxData, &mailbox);
}

/**
* @brief ���ֹͣ
*/
void motor_stop(uint8_t id)
{
  uint8_t tdata[] = {0x01, 0x00, 0x00};
  CAN_Send_Msg(0x8000 | id, tdata, sizeof(tdata));
}

/**
* @brief λ�ÿ���
* @param id ���ID
* @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
* @param ���أ���λ��0.01NM���磺torque = 110 ��ʾ�������Ϊ 1.1NM
*/
void motor_control_Pos(uint8_t id,int32_t pos,int16_t tqe)
{
  uint8_t tdata[8] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x80, 0x80, 0x00};
  tdata[2] = pos;
	tdata[3] = pos >> 8;
  tdata[6] = tqe;
	tdata[7] = tqe >> 8;
  uint32_t ext_id = (0x0000 | id);
  CAN_Send_Msg(ext_id, tdata, 8);
}

/**
* @brief �ٶȿ���
* @param id ���ID
* @param vel �ٶȣ���λ 0.00025 ת/�룬�� val = 1000 ��ʾ 0.25 ת/��
* @param tqe ���أ���λ��0.01NM����torque = 110 ��ʾ�������Ϊ1.1NM
*/
void motor_control_Vel(uint8_t id,int16_t vel,int16_t tqe)
{
  uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};
  tdata[4] = vel;
	tdata[5] = vel >> 8;
  tdata[6] = tqe;
	tdata[7] = tqe >> 8;
  uint32_t ext_id = (0x8000 | id);
  CAN_Send_Msg(ext_id, tdata, 8);
}

/**
* @brief ����ģʽ
* @param id ���ID
* @param tqe ���أ���λ��0.01NM����torque = 110 ��ʾ�������Ϊ1.1NM
*/
void motor_control_tqe(uint8_t id,int32_t tqe)
{
  uint8_t tdata[8] = {0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};
  tdata[2] = tqe;
	tdata[2] = tqe >> 8;
  CAN_Send_Msg(0x8000 | id, tdata, 4);
}

/**
* @brief ���λ��-�ٶ�-ǰ������(�������)���ƣ�int16��
* @param id ���ID
* @param tgpos λ�ã���λ 0.0001 Ȧ���� tgpos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
* @param tgval �ٶȣ���λ 0.00025 ת/�룬�� tgval = 1000 ��ʾ 0.25 ת/��
* @param tgtqe ������أ���λ��0.01 NM���� tgtqe = 110 ��ʾ�������Ϊ 1.1NM
*/
void motor_control_pos_val_tqe(uint8_t id, motor_state_s * ptr_state)
{
  static uint8_t tdata[8] = {0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int16_t tgval = (int16_t)(1000 * (ptr_state->val) / 0.25f);     //1000 * 0.25f / 0.25f = 1000
	int16_t tgtqe = (int16_t)(100 * (ptr_state->tqe));              //1.1 * 100 = 110
	int16_t tgpos = (int16_t)(10000 * (ptr_state->pos) / 2 / pi);   //10000 * 2 * pi * 0.5 /2 /pi = 5000
	
  tdata[2] = tgval;
	tdata[3] = tgval >> 8;
  tdata[4] = tgtqe;
	tdata[5] = tgtqe >> 8;
  tdata[6] = tgpos;
	tdata[7] = tgpos >> 8;
  CAN_Send_Msg(0x0000 | id, tdata, 8);
}


/**
* @brief ��ȡ���
* @param id
*/
void motor_read(uint8_t id)
{
	static uint8_t tdata[8] = {0x17, 0x01};
  CAN_Send_Msg(0x8000 | id, tdata, sizeof(tdata));

}


