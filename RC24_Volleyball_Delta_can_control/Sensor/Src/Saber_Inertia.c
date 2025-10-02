#include "Saber_Inertia.h"
#include <stdio.h>
#include <stdint.h>
#include "string.h"

uint8_t Saber_RxData[55];
Kal_Data Kalman_AccData;
Kal_Data Kalman_GyroData;
CapeEuler_Data_t CapeEuler_RxData;

void Saber_DataDecode(uint8_t * data, uint32_t sumlen);

/**************************************************************
	@brief:		BCCУ����ļ���
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
uint8_t Atom_BCC(uint8_t * addr, uint16_t len)
{
	unsigned char *DataPoint;
	DataPoint = addr;
	unsigned char XorData = 0;
	unsigned short DataLength = len;
	
	while(DataLength--)
	{
		XorData = XorData ^ *DataPoint;
		DataPoint ++;
	}
	return XorData;
}

/**************************************************************
	@brief:		����ģʽ���õĳ�ʼ��
	@param:		����ҪPID�������
	@retval: 		
	@supplement:	config��measureģʽ����
**************************************************************/
void Saber_ModeInit(Saber_Config_t * ptr, uint8_t CID, uint8_t MID)
{
	ptr -> Preamble[0] = 0x41;
	ptr -> Preamble[1] = 0x78;
	ptr -> MADDR = 0xFF;
	ptr -> CID = CID;
	ptr -> MID = MID;
	ptr -> PL = 0x00;
	ptr -> TALL = 0x6D;
}
/**************************************************************
	@brief:		ģʽ���÷��ͺ���
	@param:		1.����ҪPID�������
            2.PLһ��Ϊ0�����ֽ���8
	@retval: 		
	@supplement:	config��measureģʽ����
                configģʽ���������豸��������
                measureģʽ���豸������������������
**************************************************************/
void Saber_Mode_Set(Saber_Config_t * ptr, uint8_t CID, uint8_t MID)
{
	uint8_t txbuf[8];
	int i = 0;
	Saber_ModeInit(ptr, CID, MID);
	txbuf[i++] = ptr -> Preamble[0];
	txbuf[i++] = ptr -> Preamble[1];
	txbuf[i++] = ptr -> MADDR;
	txbuf[i++] = ptr -> CID;
	txbuf[i++] = ptr -> MID;
	txbuf[i++] = ptr -> PL;
	ptr -> Atom_BCC = Atom_BCC(txbuf, 6);
	txbuf[i++] = ptr -> Atom_BCC;
	txbuf[i++] = ptr -> TALL;
	
	HAL_UART_Transmit_DMA(&huart6, txbuf, 8);
}//config CID 01,MID 02; measure CID 01,MID 03.

/*����Ƕ����ݰ��������ã�PL�о�����ֵ��һ�����4*N,������ƴ��N��
Reserve1 + Reservel2 + PID ���ĸ��ֽڵ�����,Ȼ���У�����֡β����*/

/**************************************************************
	@brief:		�ߵ��жϻص�����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void SaberInertia_USART6_RxCpltCallBack(void)
{
	uint32_t Saber_RxLen;
	//�ж��Ƿ�Ϊ�����ж�
	if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
	{
		/*��������жϱ�־*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
		/*��ȡ���յ���GO������ݳ����뷢�͵��Ƿ�һ��*/
		Saber_RxLen = DMA_USART6_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
		if((DMA_USART6_RxBuf[0] == 'A') && (DMA_USART6_RxBuf[1] == 'x') && (DMA_USART6_RxBuf[Saber_RxLen-1] == 'm'))
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(Saber_RxData, DMA_USART6_RxBuf, Saber_RxLen);//�ӻ�������ȡ��������
			Saber_DataDecode(Saber_RxData, Saber_RxLen);
		}
		/*��������������*/
		memset(DMA_USART6_RxBuf, 0, DMA_USART6_RX_SIZE);
		/*�����趨���������ݳ���*/
		hdma_usart6_rx.Instance->NDTR = DMA_USART6_RX_SIZE;
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&hdma_usart6_rx);
	}
}

void Saber_DataDecode(uint8_t * data, uint32_t sumlen)
{
	int count = 6;
	while(sumlen - count != 6)
	{
	  uint16_t PID = data[count] | (data[++count] << 8);
	  switch(PID)
	  {
		  case(KalmanAcc_Enable_ID):
		  {
			  count = count + 2;
			  memcpy(&Kalman_AccData.Xaxis_Data, (const void * )&data[count], 12);
		  } break;
		
		  case(KalmanGyro_Enable_ID):
		  {
			  count = count + 2;
			  memcpy(&Kalman_GyroData.Xaxis_Data, (const void * )&data[count], 12);
		  } break;
		
		  case(CapeEuler_Enable_ID):
		  {
			  count = count + 2;
			  memcpy(&CapeEuler_RxData.roll, (const void * )&data[count], 12);
		  } break;
		
		  default: break;
	  }
		count = count + 12;
  }
}
