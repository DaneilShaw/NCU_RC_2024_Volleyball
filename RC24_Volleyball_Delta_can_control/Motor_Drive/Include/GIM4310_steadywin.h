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
  MOTCTRL_CMD_RESET_CONFIGURATION = 0x81,        //重置配置
  MOTCTRL_CMD_REFRESH_CONFIGURATION = 0x82,      //刷新配置
  MOTCTRL_CMD_MODIFY_CONFIGURATION = 0x83,       //修改配置项
  MOTCTRL_CMD_RETRIEVE_CONFIGURATION = 0x84,     //获取配置项
  MOTCTRL_CMD_START_MOTOR = 0x91,                //启动电机
  MOTCTRL_CMD_STOP_MOTOR = 0x92,                 //停止电机
  MOTCTRL_CMD_TORQUE_CONTROL = 0x93,             //力矩控制
  MOTCTRL_CMD_SPEED_CONTROL = 0x94,              //速度控制
  MOTCTRL_CMD_POSITION_CONTROL = 0x95,           //位置控制
  MOTCTRL_CMD_PTS_CONTROL = 0x96,                //综合控制
  MOTCTRL_CMD_STOP_CONTROL = 0x97,               //中止控制
  MOTCTRL_CMD_MODIFY_PARAMETER = 0xA1,           //修改参数项
  MOTCTRL_CMD_RETRIEVE_PARAMETER = 0xA2,         //获取参数项
  MOTCTRL_CMD_GET_VERSION = 0xB1,                //获取版本
  MOTCTRL_CMD_GET_FAULT = 0xB2,                  //获取异常
  MOTCTRL_CMD_ACK_FAULT = 0xB3,                  //消除异常
  MOTCTRL_CMD_RETRIEVE_INDICATOR = 0xB4,         //获取指标项
} MOTCTRL_CMD;    //命令字定义


typedef enum {
  MOTCTRL_RES_SUCCESS = 0,                //成功
  MOTCTRL_RES_FAIL = 1,                   //失败
  MOTCTRL_RES_FAIL_UNKNOWN_CMD = 2,       //失败，未知命令
  MOTCTRL_RES_FAIL_UNKNOWN_ID = 3,        //失败，未知ID
  MOTCTRL_RES_FAIL_RO_REG = 4,            //失败，只读寄存器
  MOTCTRL_RES_FAIL_UNKNOWN_REG = 5,       //失败，未知寄存器
  MOTCTRL_RES_FAIL_STR_FORMAT = 6,        //失败，字符串格式
  MOTCTRL_RES_FAIL_DATA_FORMAT = 7,       //失败，数据类型错误
  MOTCTRL_RES_FAIL_WO_REG = 0xB,          //失败，只写寄存器
  MOTCTRL_RES_FAIL_NOT_CONNECTED = 0x80,
} MOTCTRL_RES;   //重置配置项反馈RES含义

typedef enum {
  MOTCTRL_CONFTYPE_INT = 0,        //32位有符号整数设置
  MOTCTRL_CONFTYPE_FLOAT = 1,      //32位有符号浮点数设置
} MOTCTRL_CONFTYPE;    //修改配置项的ConfType参数设置

typedef enum {
  MOTCTRL_CONFID_INT_POLE_PAIRS = 0x00,               // 极对数
  MOTCTRL_CONFID_INT_RATED_CURRENT = 0x01,            // 额定电流 (A)
  MOTCTRL_CONFID_INT_MAX_SPEED = 0x02,                // 最大转速 (RPM)
  MOTCTRL_CONFID_INT_RATED_VOLTAGE = 0x06,            // 额定电压 (V)
  MOTCTRL_CONFID_INT_PWM_FREQ = 0x07,                 // PWM频率 (Hz)
  MOTCTRL_CONFID_INT_TORQUE_KP_DEFAULT = 0x08,        // 电流环默认KP
  MOTCTRL_CONFID_INT_TORQUE_KI_DEFAULT = 0x09,        // 电流环默认KI
  MOTCTRL_CONFID_INT_SPEED_KP_DEFAULT = 0x0C,         // 速度环默认KP
  MOTCTRL_CONFID_INT_SPEED_KI_DEFAULT = 0x0D,         // 速度环默认KI
  MOTCTRL_CONFID_INT_POSITION_KP_DEFAULT = 0x0E,      // 位置环默认KP
  MOTCTRL_CONFID_INT_POSITION_KI_DEFAULT = 0x0F,      // 位置环默认KI
  MOTCTRL_CONFID_INT_POSITION_KD_DEFAULT = 0x10,      // 位置环默认KD
  MOTCTRL_CONFID_INT_GEAR_RATIO = 0x11,               // 减速比
  MOTCTRL_CONFID_INT_CAN_ID = 0x12,                   // CAN ID
  MOTCTRL_CONFID_INT_CAN_MASTER_ID = 0x13,            // 上位机 CAN ID
  MOTCTRL_CONFID_INT_ZERO_POSITION = 0x14,            // 零点位置 (输出轴)
  MOTCTRL_CONFID_INT_POWEROFF_POSITION = 0x15,        // 断电位置 (输出轴)
  MOTCTRL_CONFID_INT_OV_THRESHOLD = 0x16,             // 过压门限值 (V)
  MOTCTRL_CONFID_INT_UV_THRESHOLD = 0x17,             // 低压门限值 (V)
  MOTCTRL_CONFID_INT_CAN_BAUDRATE = 0x18,             // CAN波特率
  MOTCTRL_CONFID_INT_FW_KP_DEFAULT = 0x19,            // 弱磁默认 KP
  MOTCTRL_CONFID_INT_FW_KI_DEFAULT = 0x1A,            // 弱磁默认 KI
  MOTCTRL_CONFID_INT_OV_TEMP_THRESHOLD = 0x20,        // 过温警告阈值
  MOTCTRL_CONFID_INT_CAN_PROT = 0x1C,                 // Protocol over CAN, @ref MOTCTRL_CONF_CAN_PROT
} MOTCTRL_CONFID_INT;   //修改配置项的有符号整数参数设置

typedef enum {
  MOTCTRL_CONFID_FLOAT_RS = 0x00,                     // 相间电阻 (Ω)
  MOTCTRL_CONFID_FLOAT_LS = 0x01,                     // 相间电感 (H)
  MOTCTRL_CONFID_FLOAT_BEMF_CONST = 0x02,             // 反电动势常数
  MOTCTRL_CONFID_FLOAT_TORQUE_CONST = 0x03,           // 转矩常数 (N.m/A)
  MOTCTRL_CONFID_FLOAT_SAMPLING_RESISTOR = 0x04,      // 采样电阻阻值 (Ω)
  MOTCTRL_CONFID_FLOAT_AMP_GAIN = 0x05,               // 采样放大增益
} MOTCTRL_CONFID_FLOAT;  //修改配置项的有符号浮点数参数设置

typedef enum {
  MOTCTRL_PARAID_TORQUE_KP = 0x00,        //电流环KP
  MOTCTRL_PARAID_TORQUE_KI = 0x01,        //电流环KI
  MOTCTRL_PARAID_SPEED_KP = 0x02,         //速度环KP
  MOTCTRL_PARAID_SPEED_KI = 0x03,         //速度环KI
  MOTCTRL_PARAID_POSITION_KP = 0x04,      //位置环KP
  MOTCTRL_PARAID_POSITION_KI = 0x05,      //位置环KI
  MOTCTRL_PARAID_POSITION_KD = 0x06,      //位置环KD
  MOTCTRL_PARAID_FW_KP = 0x07,            //弱磁KP
  MOTCTRL_PARAID_FW_KI = 0x08,            //弱磁KI
} MOTCTRL_PARAID;   //修改参数项PARAID设置


typedef enum {
  MOTCTRL_FAULTNO_NONE = 0x00,                 //无异常
  MOTCTRL_FAULTNO_FREQ_TOO_HIGH = 0x01,        //Fov频率过高
  MOTCTRL_FAULTNO_OV = 0x02,                   //过压
  MOTCTRL_FAULTNO_UV = 0x04,                   //欠压
  MOTCTRL_FAULTNO_OT = 0x08,                   //过温
  MOTCTRL_FAULTNO_START_FAIL = 0x10,           //启动失败
  MOTCTRL_FAULTNO_OC = 0x40,                   //过流
  MOTCTRL_FAULTNO_SOFTWARE_EXCEPTION = 0x80,   //软件异常
} MOTCTRL_FAULTNO;  //获取异常参数含义

typedef enum {
  MOTCTRL_INDIID_BUS_VOLTAGE = 0x00,         //母线电压(V)
  MOTCTRL_INDIID_TEMP_BOARD = 0x01,          //板载温度
  MOTCTRL_INDIID_TEMP_MOTOR = 0x02,          //电机温度
  MOTCTRL_INDIID_POWER = 0x03,               //功率(W)
  MOTCTRL_INDIID_IA = 0x04,                  //Ia电流(A)
  MOTCTRL_INDIID_IB = 0x05,                  //Ib电流(A)
  MOTCTRL_INDIID_IC = 0x06,                  //Ic电流(A)
  MOTCTRL_INDIID_IALPHA = 0x07,              //Ialpha电流(A)
  MOTCTRL_INDIID_IBETA = 0x08,               //Ibeta电流(A)
  MOTCTRL_INDIID_IQ = 0x09,                  //Iq电流(A)
  MOTCTRL_INDIID_ID = 0x0A,                  //Id电流(A)
  MOTCTRL_INDIID_TARGET_IQ = 0x0B,           //Iq目标电流(A)
  MOTCTRL_INDIID_TARGET_ID = 0x0C,           //Id目标电流(A)
  MOTCTRL_INDIID_VQ = 0x0D,                  //Vq电压(V)
  MOTCTRL_INDIID_VD = 0x0E,                  //Vd电压(V)
  MOTCTRL_INDIID_VALPHA = 0x0F,              //Valpha电压(V)
  MOTCTRL_INDIID_VBETA = 0x10,               //Vbeta电压(V)
  MOTCTRL_INDIID_EL_ANGLE_ROTOR = 0x11,      //转子电角(RAD)
  MOTCTRL_INDIID_MEC_ANGLE_ROTOR = 0x12,     //转子机械角(RAD)
  MOTCTRL_INDIID_MEC_ANGLE_SHAFT = 0x13,     //输出轴机械角(RAD)
  MOTCTRL_INDIID_SPEED_SHAFT = 0x14,         //转速(输出轴)(RPM)
  MOTCTRL_INDIID_OUTPUT_POWER = 0x15,        //输出功率(W)
} MOTCTRL_INDID;   //获取指标项IndID参数

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

