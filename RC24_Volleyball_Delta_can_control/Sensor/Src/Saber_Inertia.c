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
	@brief:		BCC校验码的计算
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
	@brief:		用于模式设置的初始化
	@param:		不需要PID相关数据
	@retval: 		
	@supplement:	config和measure模式设置
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
	@brief:		模式设置发送函数
	@param:		1.不需要PID相关数据
            2.PL一般为0，总字节数8
	@retval: 		
	@supplement:	config和measure模式设置
                config模式，主机向设备发送命令
                measure模式，设备不断向主机发送数据
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

/*如果是对数据包进行设置，PL有具体数值，一般等于4*N,即连续拼接N个
Reserve1 + Reservel2 + PID 这四个字节的数据,然后接校验码和帧尾数据*/

/**************************************************************
	@brief:		惯导中断回调函数
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void SaberInertia_USART6_RxCpltCallBack(void)
{
	uint32_t Saber_RxLen;
	//判断是否为空闲中断
	if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
	{
		/*清除接收中断标志*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		/*关闭DMA传输*/
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
		/*获取接收到的GO电机数据长度与发送的是否一致*/
		Saber_RxLen = DMA_USART6_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
		if((DMA_USART6_RxBuf[0] == 'A') && (DMA_USART6_RxBuf[1] == 'x') && (DMA_USART6_RxBuf[Saber_RxLen-1] == 'm'))
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(Saber_RxData, DMA_USART6_RxBuf, Saber_RxLen);//从缓存区获取反馈数据
			Saber_DataDecode(Saber_RxData, Saber_RxLen);
		}
		/*缓存区数据清零*/
		memset(DMA_USART6_RxBuf, 0, DMA_USART6_RX_SIZE);
		/*重新设定缓存区数据长度*/
		hdma_usart6_rx.Instance->NDTR = DMA_USART6_RX_SIZE;
		/*使能DMA*/
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
