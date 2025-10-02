#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "main.h" // stm32 hal
#include "ris_protocol.h"
#pragma pack(1)

/*****************************�ֲ�����*****************************/
//#define two_armid    0 //2�ؽ�GO���id
#define three_armid  1 //3�ؽ�GO���id
#define tail_armid   0 //β��GO���id

#define stop_mode      0 //GO���ֹͣ
#define foc_mode       1 //GO���foc����
#define calibrate_mode 2 //GO�������������


/*******************************end*******************************/

typedef union
{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

typedef struct
{
    // ���� ���ݰ�ͷ
    unsigned char start[2]; // ��ͷ
    unsigned char motorID;  // ���ID  0,1,2,3 ...   0xBB ��ʾ�����е���㲥����ʱ�޷��أ�
    unsigned char reserved;
} COMHead;

typedef struct
{ // �� 4���ֽ�һ������ ����Ȼ�����������
    // ���� ����
    uint8_t mode;       // ��ǰ�ؽ�ģʽ
    uint8_t ReadBit;    // ������Ʋ����޸�     �Ƿ�ɹ�λ
    int8_t Temp;        // �����ǰƽ���¶�
    uint8_t MError;     // ������� ��ʶ

    COMData32 Read;     // ��ȡ�ĵ�ǰ ��� �Ŀ�������
    int16_t T;          // ��ǰʵ�ʵ���������       7 + 8 ����

    int16_t W;          // ��ǰʵ�ʵ���ٶȣ����٣�   8 + 7 ����
    float LW;           // ��ǰʵ�ʵ���ٶȣ����٣�

    int16_t W2;         // ��ǰʵ�ʹؽ��ٶȣ����٣�   8 + 7 ����
    float LW2;          // ��ǰʵ�ʹؽ��ٶȣ����٣�

    int16_t Acc;        // ���ת�Ӽ��ٶ�       15+0 ����  ������С
    int16_t OutAcc;     // �������ٶ�         12+3 ����  �����ϴ�

    int32_t Pos;        // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
    int32_t Pos2;       // �ؽڱ�����λ��(���������)

    int16_t gyro[3];    // ���������6�ᴫ��������
    int16_t acc[3];

    // ��������������
    int16_t Fgyro[3];
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp;      // 8λ��ʾ���¶�  7λ��-28~100�ȣ�  1λ0.5�ȷֱ���

    int16_t Force16;    // ����������16λ����
    int8_t Force8;      // ����������8λ����

    uint8_t FError;     //  ��˴����������ʶ

    int8_t Res[1];      // ͨѶ �����ֽ�

} ServoComdV3; // �������ݰ��İ�ͷ ��CRC 78�ֽڣ�4+70+4��

typedef struct
{
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Fbk_t   fbk;    // ����������� 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} MotorData_t;  //��������

typedef struct
{
    uint8_t none[8];            // ����

} LowHzMotorCmd;

typedef struct
{                               // �� 4���ֽ�һ������ ����Ȼ�����������
                                // ���� ����
    uint8_t mode;               // �ؽ�ģʽѡ��
    uint8_t ModifyBit;          // ������Ʋ����޸�λ
    uint8_t ReadBit;            // ������Ʋ�������λ
    uint8_t reserved;

    COMData32 Modify;           // ��������޸� ������
    //ʵ�ʸ�FOC��ָ������Ϊ��
    // K_P*delta_Pos + K_W*delta_W + T
    int16_t T;                  // �����ؽڵ�������أ������������أ�x256, 7 + 8 ����
    int16_t W;                  // �����ؽ��ٶ� �����������ٶȣ� x128,       8 + 7����
    int32_t Pos;                // �����ؽ�λ�� x 16384/6.2832, 14λ������������0������������ؽڻ����Ա�����0��Ϊ׼��

    int16_t K_P;                // �ؽڸն�ϵ�� x2048  4+11 ����
    int16_t K_W;                // �ؽ��ٶ�ϵ�� x1024  5+10 ����

    uint8_t LowHzMotorCmdIndex; // ����
    uint8_t LowHzMotorCmdByte;  // ����

    COMData32 Res[1];           // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����

} MasterComdV3; // �������ݰ��İ�ͷ ��CRC 34�ֽ�

typedef struct
{
    // ���� ��������������ݰ�
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Comd_t comd;    // ����������� 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} ControlData_t;     //��������������ݰ�

#pragma pack()

typedef struct
{
    // ���� ���͸�ʽ������
    ControlData_t motor_send_data;   //����������ݽṹ��
    int hex_len;                        //���͵�16�����������鳤��, 34
    long long send_time;                //���͸������ʱ��, ΢��(us)
    // �����͵ĸ�������
    unsigned short id;                  //���ID��15��ʾ��ȫ������㲥����ʱû�����ݷ���
    unsigned short mode;                // 0:ֹͣ, 1:foc����, 2:����������궨
    //ʵ�ʸ�FOC��ָ������Ϊ��
    // K_P*delta_Pos + K_W*delta_W + T
    float T;                            //�����ؽڵ�������أ������������أ���Nm��
    float W;                            //�����ؽ��ٶȣ����������ٶȣ�(rad/s)
    float Pos;                          //�����ؽ�λ�ã�rad��
    float K_P;                          //�ؽڸն�ϵ��
    float K_W;                          //�ؽ��ٶ�ϵ��
    COMData32 Res;                    // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����
} MOTOR_send;

typedef struct
{
    // ���� ��������
    MotorData_t motor_recv_data;    //����������ݽṹ�壬���motor_msg.h
    int hex_len;                        //���յ�16�����������鳤��, 78
    long long resv_time;                //���ո������ʱ��, ΢��(us)
    int correct;                        //���������Ƿ�������1������-1��������
    //����ó��ĵ������
    unsigned char motor_id;             //���ID
    unsigned char mode;                 // 0:����, 1:foc����, 2:����������궨
    int Temp;                           //�¶�
    unsigned char MError;               //������
    float T;                            // ��ǰʵ�ʵ���������
		float W;														// speed
    float Pos;                          // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
		float footForce;										// �����ѹ���������� 12bit (0-4095)

} MOTOR_recv;


extern MOTOR_send Go_TwoArm_send;
extern MOTOR_send Go_ThreeArm_send;
extern MOTOR_send Go_TailArm_send;

extern void GoMotor_PosWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KP, float KW, float Pos);
extern void GoMotor_SpdWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KW, float W);
extern void GoMotor_DampWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float KW);
extern void GoMotor_ToqWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float T);
extern void GoMotor_ZeroToqWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode);
extern void GoMotor_ForcePos_MixWrite(MOTOR_send *motor_s, unsigned short id, unsigned short mode, float Tf, float KP, float Pos, float KD, float W);

extern void modify_data(MOTOR_send *motor_s);
extern int extract_data(MOTOR_recv *motor_r);
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);

#endif
