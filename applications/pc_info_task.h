#ifndef __PC_INFO_TASK_H
#define __PC_INFO_TASK_H
#include "stm32f4xx.h"
#include "struct_typedef.h"
#include "keyboard.h"
#define SOF_ADDR 	 0			//֡ͷ����ʽ�ֽ�ƫ����

#define FRAME_HEADER 		0x69//֡ͷ����ʽ
#define FRAME_TAILER    0x65
#define LEN_FRAME_HEADER 	1	//֡ͷ����
#define	LEN_TX_DATA 		28	//�������ݶγ���
#define	LEN_RX_DATA 		28	//�������ݶγ���
#define	LEN_FRAME_TAILER 	1	//֡βCRC16
#define	LEN_TX_PACKET		30	//���Ͱ���������
#define	LEN_RX_PACKET		33	//���հ���������

//֡ͷ�ṹ��
typedef __packed struct
{
	uint8_t  	sof;			// ֡ͷ����ʽ
	uint8_t   null_1;
	uint8_t		null_2;
	uint8_t   null_3;
} frame_header_t;

typedef __packed struct
{
	uint8_t   sof;
}frame_send_header_t;
//֡β�ṹ��
typedef __packed struct 
{
	uint8_t tailer;				// CRC16У����
} frame_tailer_t;

//�������ݽṹ��
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
	uint32_t pc_flag;
}tx_data_t;

//�������ݽṹ��
typedef __packed struct 
{
	fp32 motor1_position;
	fp32 motor2_position;
	fp32 motor3_position;
	fp32 motor4_position;
	fp32 motor5_position;
	fp32 motor6_position;
	uint32_t pc_receive_flag;
}rx_data_t;

//���Ͱ��ṹ��
typedef __packed struct 
{
	frame_send_header_t 		frame_header;	
	tx_data_t	  		tx_data;	
	frame_tailer_t 		frame_tailer;	
} send_msg_t;

//���ܰ��ṹ��
typedef __packed struct 
{
	frame_header_t	 	frame_header;	
	rx_data_t	  		rx_data;	
	frame_tailer_t 		frame_tailer;	
} receive_msg_t;

//typedef struct 
//{
//		key_t camera_1;
//		key_t camera_2;
//}pc_info_test_key;


extern send_msg_t pc_send_msg;
extern receive_msg_t pc_receive_msg;
extern void pc_info_task(void const *argu);
extern void usart2_IRQHandler(void);
#endif
