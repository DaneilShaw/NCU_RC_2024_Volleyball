#include "usart.h"
#include "GO_M8010_6.h"
#include "crc_ccitt.h"
#include "stdio.h"

MOTOR_send Go_TwoArm_send;
MOTOR_send Go_ThreeArm_send;
MOTOR_send Go_TailArm_send;

/*****************************局部函数*****************************/
uint32_t crc32_core(uint32_t *ptr, uint32_t len);
/*******************************end*******************************/

/**************************************************************
	@brief:		限定数值范围
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 


//uint32_t crc32_core(uint32_t* ptr, uint32_t len)
//{
//    uint32_t xbit = 0;
//    uint32_t data = 0;
//    uint32_t CRC32 = 0xFFFFFFFF;
//    const uint32_t dwPolynomial = 0x04c11db7;
//    for (uint32_t i = 0; i < len; i++)
//    {
//        xbit = 1 << 31;
//        data = ptr[i];
//        for (uint32_t bits = 0; bits < 32; bits++)
//        {
//            if (CRC32 & 0x80000000)
//            {
//                CRC32 <<= 1;
//                CRC32 ^= dwPolynomial;
//            }
//            else
//                CRC32 <<= 1;
//            if (data & xbit)
//                CRC32 ^= dwPolynomial;

//            xbit >>= 1;
//        }
//    }
//    return CRC32;
//}

/********* 输出 t = T + KP(Pos - 当前角度位置) + KW (w -电机转子角速度)*********/
/**************************************************************
	@brief:		GO电机位置模式设置
	@param:		6.33为减速比
	          KP为比例系数，KW为微分系数
	@retval: 		
	@supplement:	关节电机位置PD控制
**************************************************************/
void GoMotor_PosWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KP, float KW, float Pos)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = KP;/*电机阻尼系数*/
	motor_s->K_W = KW;/*电机刚度系数*/
	motor_s->T = 0.0f;/*前馈力矩*/
	motor_s->W = 0.0f;/*电机期望角速度*/
	motor_s->Pos = Pos;/*期望电机输出位置*/
}

/**************************************************************
	@brief:		GO电机速度模式设置
	@param:		6.33为减速比
            KW为比例系数
	@retval: 		
	@supplement:	关节电机转速P控制
**************************************************************/
void GoMotor_SpdWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KW, float W)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = 0.0f;
	motor_s->K_W = KW;
	motor_s->T = 0.0f;
	motor_s->W = W;
	motor_s->Pos = 0.0f;
}

/**************************************************************
	@brief:		GO电机阻尼模式设置
	@param:		KW为比例系数
	@retval: 		
	@supplement:	电机静止在当前位置
**************************************************************/
void GoMotor_DampWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KW)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = 0.0f;
	motor_s->K_W = KW;
	motor_s->T = 0.0f;
	motor_s->W = 0.0f;
	motor_s->Pos = 0.0f;
}

/**************************************************************
	@brief:		GO电机力矩模式设置
	@param:		KW为比例系数
	@retval: 		
	@supplement:	关节电机力矩控制
**************************************************************/
void GoMotor_ToqWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float T)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = 0.0f;
	motor_s->K_W = 0.0f;
	motor_s->T = T;
	motor_s->W = 0.0f;
	motor_s->Pos = 0.0f;
}

/**************************************************************
	@brief:		GO电机零力矩模式设置
	@param:		
	@retval: 		
	@supplement:	主动产生力矩抵抗自身产生摩擦力，外部力量可以使其转动
**************************************************************/
void GoMotor_ZeroToqWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = 0.0f;
	motor_s->K_W = 0.0f;
	motor_s->T = 0.0f;
	motor_s->W = 0.0f;
	motor_s->Pos = 0.0f;
}

/**************************************************************
	@brief:		GO电机力位混合模式设置
	@param:		
	@retval: 		
	@supplement:	同时对转速，位置，力矩进行控制
**************************************************************/
void GoMotor_ForcePos_MixWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float Tf, float KP, float Pos, float KD, float W)
{
	motor_s->id = id;
	motor_s->mode = mode;
	motor_s->K_P = KP;
	motor_s->K_W = KD;
	motor_s->T = Tf;
	motor_s->W = W;
	motor_s->Pos = Pos;
}

/**************************************************************
	@brief:		GO数据发送
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void modify_data(MOTOR_send *motor_s)
{
	/*控制电机时，通过串口给电机发送一个长度为17（Byte）字节的数据*/
  motor_s->hex_len = 17;
	
	/*数据包头 2Byte*/
  motor_s->motor_send_data.head[0] = 0xFE;
  motor_s->motor_send_data.head[1] = 0xEE;

	/*说明书里的默认输出限幅*/	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
	
	SATURATE(motor_s->K_P,  0.0f,   25.599f);/*电机阻尼系数*/
	SATURATE(motor_s->K_W,  0.0f,   25.599f);/*电机刚度系数*/
	SATURATE(motor_s->T,   -127.99f,  127.99f);/*期望电机转矩*/
	SATURATE(motor_s->W,   -804.00f,  804.00f);/*电机期望角速度*/
	SATURATE(motor_s->Pos, -411774.0f,  411774.0f);	/*期望电机输出位置*/
	
	/*模式设置 1Byte*/
  motor_s->motor_send_data.mode.id   = motor_s->id;
  motor_s->motor_send_data.mode.status  = motor_s->mode;
	
	/*参数设置 k_pos, k_spd, pos_des, spd_des ,tor_des 五个参数各占2Byte*/
  motor_s->motor_send_data.comd.k_pos  = motor_s->K_P * 1280;//对motor_s -> K_P/25.6f*32768进行化简
  motor_s->motor_send_data.comd.k_spd  = motor_s->K_W * 1280;//对motor_s -> K_W/25.6f*32768进行化简
  motor_s->motor_send_data.comd.pos_des  = 207421.44f * motor_s->Pos / 6.2832f;//代入6.33,即32768*6.33=27421.44, 6.2832 == 2PI ,转化为输出端控制送
	  /*代入6.33, 256*6.33/2PI = 257.907(PI = 3.1415),转化为输出端控制
	   1620.48f * motor_s->W / 6.2832f公式中motor_s->W为输入角速度*/
  motor_s->motor_send_data.comd.spd_des  = 257.907f * motor_s->W;
  motor_s->motor_send_data.comd.tor_des  = motor_s->T * 256;
	
	/*校验部分 CRC16 2Byte*/
  motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
	
	/*数据发送*/
  HAL_UART_Transmit(&huart6, (uint8_t *)motor_s, sizeof(motor_s->motor_send_data), 10);
}

/**************************************************************
	@brief:		数据解算
	@param:		
	@retval: 		
	@supplement:
**************************************************************/
int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
		{
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256 / 6.33f) * 6.2832f ;//转化为输出端速度
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768 / 6.33f;//转化为输出端位置
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
        return motor_r->correct;
    }
}

//HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
//{
//    uint16_t rxlen = 0;

//    modify_data(pData);
//    
//    HAL_UART_Transmit(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 

//    if(rxlen == 0)

//      return HAL_TIMEOUT;

//    if(rxlen != sizeof(rData->motor_recv_data))
//			return HAL_ERROR;

//    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
//    if(rp[0] == 0xFE && rp[1] == 0xEE)
//    {
//        rData->correct = 1;
//        extract_data(rData);
//        return HAL_OK;
//    }
//    
//    return HAL_ERROR;
//}
