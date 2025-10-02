#ifndef GIM4310_STEADYWIN_H
#define GIM4310_STEADYWIN_H

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>
#include "struct_typedef.h"

#define MOTCTRL_MOTOR_SIZE 8
#define Steady_Motor_ID    0x01

typedef enum {
  MOTCTRL_CMD_RESET_CONFIGURATION = 0x81,        //��������
  MOTCTRL_CMD_REFRESH_CONFIGURATION = 0x82,      //ˢ������
  MOTCTRL_CMD_MODIFY_CONFIGURATION = 0x83,       //�޸�������
  MOTCTRL_CMD_RETRIEVE_CONFIGURATION = 0x84,     //��ȡ������
  MOTCTRL_CMD_START_MOTOR = 0x91,                //�������
  MOTCTRL_CMD_STOP_MOTOR = 0x92,                 //ֹͣ���
  MOTCTRL_CMD_TORQUE_CONTROL = 0x93,             //���ؿ���
  MOTCTRL_CMD_SPEED_CONTROL = 0x94,              //�ٶȿ���
  MOTCTRL_CMD_POSITION_CONTROL = 0x95,           //λ�ÿ���
  MOTCTRL_CMD_PTS_CONTROL = 0x96,                //�ۺϿ���
  MOTCTRL_CMD_STOP_CONTROL = 0x97,               //��ֹ����
  MOTCTRL_CMD_MODIFY_PARAMETER = 0xA1,           //�޸Ĳ�����
  MOTCTRL_CMD_RETRIEVE_PARAMETER = 0xA2,         //��ȡ������
  MOTCTRL_CMD_GET_VERSION = 0xB1,                //��ȡ�汾
  MOTCTRL_CMD_GET_FAULT = 0xB2,                  //��ȡ�쳣
  MOTCTRL_CMD_ACK_FAULT = 0xB3,                  //�����쳣
  MOTCTRL_CMD_RETRIEVE_INDICATOR = 0xB4,         //��ȡָ����
} MOTCTRL_CMD;    //�����ֶ���


typedef enum {
  MOTCTRL_RES_SUCCESS = 0,                //�ɹ�
  MOTCTRL_RES_FAIL = 1,                   //ʧ��
  MOTCTRL_RES_FAIL_UNKNOWN_CMD = 2,       //ʧ�ܣ�δ֪����
  MOTCTRL_RES_FAIL_UNKNOWN_ID = 3,        //ʧ�ܣ�δ֪ID
  MOTCTRL_RES_FAIL_RO_REG = 4,            //ʧ�ܣ�ֻ���Ĵ���
  MOTCTRL_RES_FAIL_UNKNOWN_REG = 5,       //ʧ�ܣ�δ֪�Ĵ���
  MOTCTRL_RES_FAIL_STR_FORMAT = 6,        //ʧ�ܣ��ַ�����ʽ
  MOTCTRL_RES_FAIL_DATA_FORMAT = 7,       //ʧ�ܣ��������ʹ���
  MOTCTRL_RES_FAIL_WO_REG = 0xB,          //ʧ�ܣ�ֻд�Ĵ���
  MOTCTRL_RES_FAIL_NOT_CONNECTED = 0x80,
} MOTCTRL_RES;   //�����������RES����

typedef enum {
  MOTCTRL_CONFTYPE_INT = 0,        //32λ�з�����������
  MOTCTRL_CONFTYPE_FLOAT = 1,      //32λ�з��Ÿ���������
} MOTCTRL_CONFTYPE;    //�޸��������ConfType��������

typedef enum {
  MOTCTRL_CONFID_INT_POLE_PAIRS = 0x00,               // ������
  MOTCTRL_CONFID_INT_RATED_CURRENT = 0x01,            // ����� (A)
  MOTCTRL_CONFID_INT_MAX_SPEED = 0x02,                // ���ת�� (RPM)
  MOTCTRL_CONFID_INT_RATED_VOLTAGE = 0x06,            // ���ѹ (V)
  MOTCTRL_CONFID_INT_PWM_FREQ = 0x07,                 // PWMƵ�� (Hz)
  MOTCTRL_CONFID_INT_TORQUE_KP_DEFAULT = 0x08,        // ������Ĭ��KP
  MOTCTRL_CONFID_INT_TORQUE_KI_DEFAULT = 0x09,        // ������Ĭ��KI
  MOTCTRL_CONFID_INT_SPEED_KP_DEFAULT = 0x0C,         // �ٶȻ�Ĭ��KP
  MOTCTRL_CONFID_INT_SPEED_KI_DEFAULT = 0x0D,         // �ٶȻ�Ĭ��KI
  MOTCTRL_CONFID_INT_POSITION_KP_DEFAULT = 0x0E,      // λ�û�Ĭ��KP
  MOTCTRL_CONFID_INT_POSITION_KI_DEFAULT = 0x0F,      // λ�û�Ĭ��KI
  MOTCTRL_CONFID_INT_POSITION_KD_DEFAULT = 0x10,      // λ�û�Ĭ��KD
  MOTCTRL_CONFID_INT_GEAR_RATIO = 0x11,               // ���ٱ�
  MOTCTRL_CONFID_INT_CAN_ID = 0x12,                   // CAN ID
  MOTCTRL_CONFID_INT_CAN_MASTER_ID = 0x13,            // ��λ�� CAN ID
  MOTCTRL_CONFID_INT_ZERO_POSITION = 0x14,            // ���λ�� (�����)
  MOTCTRL_CONFID_INT_POWEROFF_POSITION = 0x15,        // �ϵ�λ�� (�����)
  MOTCTRL_CONFID_INT_OV_THRESHOLD = 0x16,             // ��ѹ����ֵ (V)
  MOTCTRL_CONFID_INT_UV_THRESHOLD = 0x17,             // ��ѹ����ֵ (V)
  MOTCTRL_CONFID_INT_CAN_BAUDRATE = 0x18,             // CAN������
  MOTCTRL_CONFID_INT_FW_KP_DEFAULT = 0x19,            // ����Ĭ�� KP
  MOTCTRL_CONFID_INT_FW_KI_DEFAULT = 0x1A,            // ����Ĭ�� KI
  MOTCTRL_CONFID_INT_OV_TEMP_THRESHOLD = 0x20,        // ���¾�����ֵ
  MOTCTRL_CONFID_INT_CAN_PROT = 0x1C,                 // Protocol over CAN, @ref MOTCTRL_CONF_CAN_PROT
} MOTCTRL_CONFID_INT;   //�޸���������з���������������

typedef enum {
  MOTCTRL_CONFID_FLOAT_RS = 0x00,                     // ������ (��)
  MOTCTRL_CONFID_FLOAT_LS = 0x01,                     // ����� (H)
  MOTCTRL_CONFID_FLOAT_BEMF_CONST = 0x02,             // ���綯�Ƴ���
  MOTCTRL_CONFID_FLOAT_TORQUE_CONST = 0x03,           // ת�س��� (N.m/A)
  MOTCTRL_CONFID_FLOAT_SAMPLING_RESISTOR = 0x04,      // ����������ֵ (��)
  MOTCTRL_CONFID_FLOAT_AMP_GAIN = 0x05,               // �����Ŵ�����
} MOTCTRL_CONFID_FLOAT;  //�޸���������з��Ÿ�������������

typedef enum {
  MOTCTRL_PARAID_TORQUE_KP = 0x00,        //������KP
  MOTCTRL_PARAID_TORQUE_KI = 0x01,        //������KI
  MOTCTRL_PARAID_SPEED_KP = 0x02,         //�ٶȻ�KP
  MOTCTRL_PARAID_SPEED_KI = 0x03,         //�ٶȻ�KI
  MOTCTRL_PARAID_POSITION_KP = 0x04,      //λ�û�KP
  MOTCTRL_PARAID_POSITION_KI = 0x05,      //λ�û�KI
  MOTCTRL_PARAID_POSITION_KD = 0x06,      //λ�û�KD
  MOTCTRL_PARAID_FW_KP = 0x07,            //����KP
  MOTCTRL_PARAID_FW_KI = 0x08,            //����KI
} MOTCTRL_PARAID;   //�޸Ĳ�����PARAID����


typedef enum {
  MOTCTRL_FAULTNO_NONE = 0x00,                 //���쳣
  MOTCTRL_FAULTNO_FREQ_TOO_HIGH = 0x01,        //FovƵ�ʹ���
  MOTCTRL_FAULTNO_OV = 0x02,                   //��ѹ
  MOTCTRL_FAULTNO_UV = 0x04,                   //Ƿѹ
  MOTCTRL_FAULTNO_OT = 0x08,                   //����
  MOTCTRL_FAULTNO_START_FAIL = 0x10,           //����ʧ��
  MOTCTRL_FAULTNO_OC = 0x40,                   //����
  MOTCTRL_FAULTNO_SOFTWARE_EXCEPTION = 0x80,   //����쳣
} MOTCTRL_FAULTNO;  //��ȡ�쳣��������

typedef enum {
  MOTCTRL_INDIID_BUS_VOLTAGE = 0x00,         //ĸ�ߵ�ѹ(V)
  MOTCTRL_INDIID_TEMP_BOARD = 0x01,          //�����¶�
  MOTCTRL_INDIID_TEMP_MOTOR = 0x02,          //����¶�
  MOTCTRL_INDIID_POWER = 0x03,               //����(W)
  MOTCTRL_INDIID_IA = 0x04,                  //Ia����(A)
  MOTCTRL_INDIID_IB = 0x05,                  //Ib����(A)
  MOTCTRL_INDIID_IC = 0x06,                  //Ic����(A)
  MOTCTRL_INDIID_IALPHA = 0x07,              //Ialpha����(A)
  MOTCTRL_INDIID_IBETA = 0x08,               //Ibeta����(A)
  MOTCTRL_INDIID_IQ = 0x09,                  //Iq����(A)
  MOTCTRL_INDIID_ID = 0x0A,                  //Id����(A)
  MOTCTRL_INDIID_TARGET_IQ = 0x0B,           //IqĿ�����(A)
  MOTCTRL_INDIID_TARGET_ID = 0x0C,           //IdĿ�����(A)
  MOTCTRL_INDIID_VQ = 0x0D,                  //Vq��ѹ(V)
  MOTCTRL_INDIID_VD = 0x0E,                  //Vd��ѹ(V)
  MOTCTRL_INDIID_VALPHA = 0x0F,              //Valpha��ѹ(V)
  MOTCTRL_INDIID_VBETA = 0x10,               //Vbeta��ѹ(V)
  MOTCTRL_INDIID_EL_ANGLE_ROTOR = 0x11,      //ת�ӵ��(RAD)
  MOTCTRL_INDIID_MEC_ANGLE_ROTOR = 0x12,     //ת�ӻ�е��(RAD)
  MOTCTRL_INDIID_MEC_ANGLE_SHAFT = 0x13,     //������е��(RAD)
  MOTCTRL_INDIID_SPEED_SHAFT = 0x14,         //ת��(�����)(RPM)
  MOTCTRL_INDIID_OUTPUT_POWER = 0x15,        //�������(W)
} MOTCTRL_INDID;   //��ȡָ����IndID����

typedef struct {
	uint8_t motctrl_cmd;
	uint8_t res;
	int8_t temperature;
	float position;
	float speed;
	float toque;
	uint8_t IndID;
}Motctrl_Rx;

extern float Steadymotor_Basepos;
//extern uint16_t steady_motorcnt; 

void MXReqStop(void);
void MXReqStart(void);
void MXReqSpeedControl(float speed, uint32_t duration);
void MCReqPositionControl(float position, uint32_t duration);
void MXReqToqueControl(float speed,uint32_t duration);
void MXReqCancel(void);
void MXReqInquire(uint8_t IndID);

#endif

