#ifndef BSP_RS485_H
#define BSP_RS485_H

#include "struct_typedef.h"

#define REDUCTION_RATIO  9.1f
#define CriticalVelocity 1.5f
#define CriticalTorque   0.01f
#define StartupTorque    0.05f
#define PI					3.14159265358979f

typedef int16_t q15_t;

#pragma pack(1)//�����ڴ����Ϊ��1�ֽڶ���

// �����õ����������ݽṹ
typedef union
{
        int32_t           L;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

// ���� ���ݰ�ͷ
typedef struct
{
		unsigned char  start[2];     // ��ͷ
		unsigned char  motorID;      // ���ID  0,1,2,3 ...   0xBB ��ʾ�����е���㲥����ʱ�޷��أ�
		unsigned char  reserved;
}COMHead;

#pragma pack()//ȡ��ָ�����룬�ָ�ȱʡ����

#pragma pack(1)//�����ڴ����Ϊ��1�ֽڶ���

// ���� ����
typedef struct 
{  // �� 4���ֽ�һ������ ����Ȼ�����������
    uint8_t  mode;        // �ؽ�ģʽѡ��
    uint8_t  ModifyBit;   // ������Ʋ����޸�λ
    uint8_t  ReadBit;     // ������Ʋ�������λ
    uint8_t  reserved;

    COMData32  Modify;     // ��������޸� ������ 
    //ʵ�ʸ�FOC��ָ������Ϊ��
    //K_P*delta_Pos + K_W*delta_W + T
    q15_t     T;      // �����ؽڵ�������أ������������أ�x256, 7 + 8 ����
    q15_t     W;      // �����ؽ��ٶ� �����������ٶȣ� x128,       8 + 7����	
    int32_t   Pos;      // �����ؽ�λ�� x 16384/6.2832, 14λ������������0������������ؽڻ����Ա�����0��Ϊ׼��

    q15_t    K_P;      // �ؽڸն�ϵ�� x2048  4+11 ����
    q15_t    K_W;      // �ؽ��ٶ�ϵ�� x1024  5+10 ����

    uint8_t LowHzMotorCmdIndex;     // �����Ƶ�ʿ������������, 0-7, �ֱ����LowHzMotorCmd�е�8���ֽ�
    uint8_t LowHzMotorCmdByte;      // �����Ƶ�ʿ���������ֽ�
	
    COMData32  Res[1];    // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����
	
}MasterComdV3;   // �������ݰ��İ�ͷ ��CRC 34�ֽ�

// ���� ��������������ݰ�	
typedef struct 
{
    COMHead 			head;    
    MasterComdV3  Mdata;
    COMData32 		CRCdata;
}MasterComdDataV3;//��������

#pragma pack()//ȡ��ָ�����룬�ָ�ȱʡ����



#pragma pack(1)//�����ڴ����Ϊ��1�ֽڶ���

typedef struct {  // �� 4���ֽ�һ������ ����Ȼ�����������
    // ���� ����
    uint8_t  mode;        // ��ǰ�ؽ�ģʽ
    uint8_t  ReadBit;     // ������Ʋ����޸�     �Ƿ�ɹ�λ
    int8_t   Temp;        // �����ǰƽ���¶�   
    uint8_t  MError;      // ������� ��ʶ
 
    COMData32  Read;     // ��ȡ�ĵ�ǰ ��� �Ŀ������� 
    int16_t     T;      // ��ǰʵ�ʵ���������       7 + 8 ����

    int16_t     W;      // ��ǰʵ�ʵ���ٶȣ����٣�   8 + 7 ����
    float      LW;      // ��ǰʵ�ʵ���ٶȣ����٣�   

    int16_t     W2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   8 + 7 ����
    float      LW2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   

    int16_t    Acc;           // ���ת�Ӽ��ٶ�       15+0 ����  ������С
    int16_t    OutAcc;        // �������ٶ�         12+3 ����  �����ϴ�
		 
    int32_t   Pos;      // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
    int32_t   Pos2;     // �ؽڱ�����λ��(���������)

    int16_t     gyro[3];  // ���������6�ᴫ��������
    int16_t     acc[3];   

    // ��������������   
    int16_t     Fgyro[3];  //  
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;     // 8λ��ʾ���¶�  7λ��-28~100�ȣ�  1λ0.5�ȷֱ���
    
    int16_t     Force16;   // ����������16λ����
    int8_t      Force8;    // ����������8λ����
		
    uint8_t     FError;    //  ��˴����������ʶ
		
    int8_t      Res[1];    // ͨѶ �����ֽ�
	
}ServoComdV3;  // �������ݰ��İ�ͷ ��CRC 78�ֽڣ�4+70+4��

typedef struct {
    // ���� ��������������ݰ�	
    COMHead        	head;
    ServoComdV3     Mdata;
    COMData32    		CRCdata;

}ServoComdDataV3;	//��������

#pragma pack()//ȡ��ָ�����룬�ָ�ȱʡ����

//���巢������
typedef struct {
    MasterComdDataV3  motor_send_data;  //����������ݽṹ�壬���motor_msg.h
		int hex_len;                    //���������ֽ���, �����ӦΪ34
    long long send_time;            //���͸������ʱ��, ΢��(us)
    
		//�����͵ĸ�������
    unsigned short id;              //���ID
    unsigned short mode;            //�������ģʽ��0:����, 5:��������ת��, 10:�ջ�����
    //���²�����Ϊ��������������������޹ء�ʵ�ʴ��ݸ����ư��ָ������Ϊ��
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //������������������أ�Nm��
    float W;                        //�������������ٶ�(rad/s)
    float Pos;                      //������������λ�ã�rad��
    float K_P;                      //��������λ�øն�ϵ��
    float K_W;                      //���������ٶȸն�ϵ��
}MOTOR_send;

//����������ݽṹ
typedef struct
{
    ServoComdDataV3 motor_recv_data;     //The data received from motor. Details are shown in motor_msg.h������������ݽṹ�壬���motor_msg.h��
    int hex_len;                    //The Bytes count of the received message, it should be 78 for this motor��������Ϣ���ֽ���, �����ӦΪ78��
    long long resv_time;            //The time of receiving this message, microsecond(us)�����ո������ʱ��, ΢��(us)��
    int correct;                    //Whether the received data is correct(1:correct, 0:wrong)�����������Ƿ�������1������0����������
    
	//����ó��ĵ������
    unsigned char motor_id;         //Motor ID�����ID��
    unsigned char mode;             //The control mode, 0:free, 5:Open loop slow turning, 10:close loop control���������ģʽ��0:����, 5:��������ת��, 10:�ջ����ơ�
    int 					Temp;                       //Temperature���¶ȡ�
    unsigned char MError;           //Error code�������롿

    float T;                        //The output torque of motor����ǰʵ�ʵ��������ء�
    float W;                        //The motor shaft speed(without filter)��ǰʵ�ʵ���ٶȣ����˲�������
    float LW;                       //The motor shaft speed(with filter)����ǰʵ�ʵ���ٶȣ����˲�������
    int   Acc;                        //The acceleration of motor shaft�����ת�Ӽ��ٶȡ�
    float Pos;                      //The motor shaft position(control board zero fixed)����ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼����
	
    float gyro[3];                  //The data of 6 axis inertial sensor on the control board�����������6�ᴫ�������ݡ�
    float acc[3];

}MOTOR_recv;


uint32_t crc32_core(uint32_t* ptr, uint32_t len);
void extract_data(MOTOR_recv * motor);
extern void rs485_uart_init(void);
extern void modify_and_send_data_0(MOTOR_send * motor);
extern void modify_and_send_data_1(MOTOR_send * motor);
extern void modify_and_send_data_2(MOTOR_send * motor);
extern void modify_and_send_data_3(MOTOR_send * motor);
extern float A1_Friction_Compensation(MOTOR_recv *motor, float torque);
#endif


