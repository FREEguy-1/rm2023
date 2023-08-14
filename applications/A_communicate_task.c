#include "A_communicate_task.h"
#include "usart.h"
#include "pc_info_task.h"
#include "chassis_task.h"
#include "air_control_task.h"								
#include "cmsis_os.h"
#include "main.h"
#include "bsp_usart.h"
#include "arm_control_task.h"
#include "string.h"


/*发送编码与接收编码函数*/
static void A_A_init(void);
static void key_board_code(chassis_move_t *chassis_move_key_board);
static void A_A_TXcode(uint8_t key_message, int16_t ch_message, fp32 motor_1, fp32 motor_2, fp32 motor_3, fp32 motor_4, fp32 motor_5);

/*发送信息数组与接收信息数组*/
uint8_t tx_message[30];
uint8_t rx_message[27];
uint8_t is_right_position; //判断机械臂电机是否移动到目标位置
fp32 rx_motor_position[5];  //当前电机反馈角度
uint8_t key_message;
int16_t ch_message;
int16_t receive_calculate;
uint8_t receive_flag;
uint8_t which_key_press;
fp32 arm_4_observe;
fp32 arm_5_observe;
fp32 temp;

void A_communicate_task(void const * argument)
{
	A_A_init();
	while(1)
	{
		key_board_code(&chassis_move); //对当前按下的按键进行编码
//		A_A_TXcode(key_message, ch_message, pc_receive_msg.rx_data.motor1_position, pc_receive_msg.rx_data.motor2_position, 
//										pc_receive_msg.rx_data.motor3_position, pc_receive_msg.rx_data.motor4_position, pc_receive_msg.rx_data.motor5_position); //计算发送数据
		if(mode_control_flag == SECOND_MODE)
		{
//			if (chassis_move.chassis_RC->key.CTRL == 0)
//			{
//				A_A_TXcode(key_message,ch_message,pc_receive_msg.rx_data.motor2_position,pc_receive_msg.rx_data.motor3_position,pc_receive_msg.rx_data.motor4_position,pc_receive_msg.rx_data.motor5_position,pc_receive_msg.rx_data.motor6_position+3.14f);
//			}
//			else
//			{
//				A_A_TXcode(key_message,ch_message,pc_receive_msg.rx_data.motor2_position,pc_receive_msg.rx_data.motor3_position,pc_receive_msg.rx_data.motor4_position+arm_4_observe,pc_receive_msg.rx_data.motor5_position+arm_5_observe,pc_receive_msg.rx_data.motor6_position+3.14f);
//			}
			A_A_TXcode(key_message,ch_message,arm_2_set,arm_3_set,arm_control.motor_4_position+arm_4_compansate,arm_control.motor_5_position+arm_5_compansate,arm_control.motor_6_position+arm_6_compansate);
		}
		else if(mode_control_flag == FIRST_MODE)
		{
			if(chassis_move.chassis_RC->key.CTRL == 1)
			{
					A_A_TXcode(key_message,ch_message,arm_control.motor_2_position,arm_control.motor_3_position,arm_control.motor_4_position,arm_control.motor_5_position+arm_5_compansate,arm_control.motor_6_position);
			}
			else
			{
			   A_A_TXcode(key_message,ch_message,arm_control.motor_2_position,arm_control.motor_3_position,arm_control.motor_4_position,arm_control.motor_5_position,arm_control.motor_6_position);
			}
		}
		else if(mode_control_flag == THIRD_MODE)
		{
			if (chassis_move.chassis_RC->key.CTRL == 0)
			{
				A_A_TXcode(key_message,ch_message,pc_receive_msg.rx_data.motor2_position,pc_receive_msg.rx_data.motor3_position,pc_receive_msg.rx_data.motor4_position,pc_receive_msg.rx_data.motor5_position,pc_receive_msg.rx_data.motor6_position+3.14f);
			}
			else
			{
				A_A_TXcode(key_message,ch_message,arm_2_set,arm_3_set,pc_receive_msg.rx_data.motor4_position+arm_4_compansate,pc_receive_msg.rx_data.motor5_position+arm_5_compansate,pc_receive_msg.rx_data.motor6_position+arm_6_compansate+3.14f);
			}
		}
		HAL_UART_Transmit(&huart7, tx_message, 30, 500); //发送数据
		osDelay(2);
	}
}

/*初始化先将所有数据设置为0*/
void A_A_init(void)
{
	for(int i = 0; i < 17; i++)
	{
		rx_message[i] = 0;
	}
	for(int i = 0; i < 19; i++)
	{
		tx_message[i] = 0;
	}
	for(int i = 0; i < 5; i++)
	{
		rx_motor_position[i] = 0.0f;
	}
	is_right_position = 0;
	key_message = 0;
	ch_message = 0;
	receive_flag = 0;
	receive_calculate = 0;
	arm_4_observe = 0.0f;
	arm_5_observe = 0.0f;
}

/*对当前按下的按键进行检测编码*/
void  key_board_code(chassis_move_t *chassis_move_key_board)
{
	if(chassis_move_key_board->chassis_RC == NULL)
	{
		return;
	}
	
	if(switch_is_down(chassis_move.chassis_RC->rc.s[0]))
	{
		key_message = 0;
		ch_message = chassis_move_key_board->chassis_RC->rc.ch[1];
	}
	else if(mode_control_flag == FIRST_MODE)
	{
		key_message = 1;
	}
	else if(mode_control_flag == SECOND_MODE || mode_control_flag == THIRD_MODE)
	{
		key_message = 2;
		if(chassis_move_key_board->chassis_RC->key.Z == 1)
		{
			which_key_press = 1;
		}
		else if(chassis_move_key_board->chassis_RC->key.X == 1)
		{
			which_key_press = 2;
		}
		else if(chassis_move_key_board->chassis_RC->key.C == 1)
		{
			which_key_press = 3;
		}
		else if(chassis_move_key_board->chassis_RC->key.V == 1)
		{
			which_key_press = 4;
		}
		else if(chassis_move_key_board->chassis_RC->key.B == 1)
		{
			which_key_press = 5;
		}
		else
		{
			which_key_press = 0;
		}
		ch_message = chassis_move_key_board->chassis_RC->rc.ch[1];
	}
	else
	{
		key_message = 0;
		ch_message = 0;
		which_key_press = 0;
	}
}
/*整合要发送的数据*/
void A_A_TXcode(uint8_t key_message, int16_t ch_message, fp32 motor_1, fp32 motor_2, fp32 motor_3, fp32 motor_4, fp32 motor_5)
{
	uint8_t *position_buf;
	tx_message[0] = 0xFA;
	tx_message[1] = 0xFB;
	tx_message[2] = (uint8_t)(key_message&0xFF);
	tx_message[3] = (uint8_t)(which_key_press&0xFF);
	tx_message[4] = (uint8_t)((ch_message>>8)&0xFF);
	tx_message[5] = (uint8_t)(ch_message&0xFF);
	tx_message[6] = 0xFC;
	tx_message[7] = 0xFD;
	position_buf = (uint8_t*) &motor_1;
	tx_message[8] = *position_buf;
	tx_message[9] = *(position_buf+1);
	tx_message[10] = *(position_buf+2);
	tx_message[11] = *(position_buf+3);	
	position_buf = (uint8_t*) &motor_2;
	tx_message[12] = *position_buf;
	tx_message[13] = *(position_buf+1);	
	tx_message[14] = *(position_buf+2);
	tx_message[15] = *(position_buf+3);
	position_buf = (uint8_t*) &motor_3;
	tx_message[16] = *position_buf;
	tx_message[17] = *(position_buf+1);	
	tx_message[18] = *(position_buf+2);
	tx_message[19] = *(position_buf+3);
	position_buf = (uint8_t*) &motor_4;
	tx_message[20] = *position_buf;
	tx_message[21] = *(position_buf+1);	
	tx_message[22] = *(position_buf+2);
	tx_message[23] = *(position_buf+3);
	position_buf = (uint8_t*) &motor_5;
	tx_message[24] = *position_buf;
	tx_message[25] = *(position_buf+1);	
	tx_message[26] = *(position_buf+2);
	tx_message[27] = *(position_buf+3);
	tx_message[28] = 0xFE;
	tx_message[29] = 0xFF;
}


void UART7_IRQHandler(void)  
{
    volatile uint8_t receive;
    //receive interrupt 接收中断
    if(huart7.Instance->SR & UART_FLAG_RXNE)
    {
			receive = huart7.Instance->DR;
			if(receive == 0xFA)
			{
				receive_calculate = 0;
				receive_flag = 1;
			}
			if(receive == 0xFB)
			{
				if(receive_calculate != 1)
				{
					receive_calculate = 0;
					receive_flag = 0;
				}
			}
			else if(receive == 0xFD)
			{
				if(rx_message[receive_calculate-1]!=0xFC || receive_calculate != 4)
				{
					receive_flag = 0;
					receive_calculate = 0;
				}
			}
			else if(receive == 0xFF)
			{
				if(rx_message[receive_calculate -1] != 0xFE || receive_calculate != 26)
				{
					receive_flag = 0;
					receive_calculate = 0;
				}
			}
			if(receive_calculate == 27)
			{
				receive_calculate = 0;
				receive_flag = 0;
			}
			
			if(receive_flag == 1)
			{
				rx_message[receive_calculate] = receive;
				receive_calculate++;
			}
			
			if(rx_message[0] == 0xFA && rx_message[1] == 0xFB && rx_message[3] == 0xFC && rx_message[4] == 0xFD && rx_message[25] == 0xFE && rx_message[26] == 0xFF)
			{
//				fp32* pos_buf;
				is_right_position = 0;
				receive_calculate = 0;
//				is_right_position = rx_message[2];
//				pos_buf = (fp32*) &rx_message[5];
//				rx_motor_position[0] = *pos_buf;
//				pos_buf = (fp32*) &rx_message[9];
//				rx_motor_position[1] = *pos_buf;
//				pos_buf = (fp32*) &rx_message[13];
//				rx_motor_position[2] = *pos_buf;
//				pos_buf = (fp32*) &rx_message[17];
//				rx_motor_position[3] = *pos_buf;
//				pos_buf = (fp32*) &rx_message[21];
//				rx_motor_position[4] = *pos_buf;
				memcpy(&rx_motor_position[0], &rx_message[5],4);
				memcpy(&rx_motor_position[1], &rx_message[9],4);
				memcpy(&rx_motor_position[2], &rx_message[13],4);
				memcpy(&rx_motor_position[3], &rx_message[17],4);
				memcpy(&rx_motor_position[4], &rx_message[21],4);
				for(int i = 5 ; i<27; i = i+1)
				{
					rx_message[i] = 0;
				}
				receive_flag = 0;
			}
		}
		else
		{
			uart7_init();
		}
}

