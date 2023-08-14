#include "pc_info_task.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "string.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_zero_task.h"
#include "A_communicate_task.h"

send_msg_t pc_send_msg;
//pc_info_test_key test_key;

receive_msg_t pc_receive_msg;
uint8_t PC_SEND_BUF[LEN_TX_PACKET + 1];
uint8_t seqcount_1 = 0;
static void data_solve(receive_msg_t *pc_receive_msg, uint8_t *rx_data);


void pc_info_task(void const *argu)
{
    //获取当前系统时间，方便延时
	uint32_t mode_wake_time = osKernelSysTick();
	pc_send_msg.tx_data.pc_flag = 0;
	while(1)
	{	
		//对各数据赋值
		pc_send_msg.frame_header.sof = 0x69;
//		pc_send_msg.tx_data.cameraType = servo_key.lift_down_1_key.itself.mode;
		//在帧头和帧尾加入CRC校验，其中CRC8校验函数可以不调用
//		append_crc8_check_sum(&pc_send_msg.frame_header.sof, LEN_FRAME_HEADER);
//		append_crc16_check_sum(&pc_send_msg.frame_header.sof, LEN_TX_PACKET);
		//发送数据
		if(pc_receive_msg.rx_data.pc_receive_flag == 3)
		{
			pc_send_msg.tx_data.pc_flag = 0;
		}
		pc_send_msg.tx_data.motor1_position = -(arm_move.arm_motor.angle - motor_angle_init)/1.5f;
		pc_send_msg.tx_data.motor2_position = rx_motor_position[0];
		pc_send_msg.tx_data.motor3_position = rx_motor_position[1];
		pc_send_msg.tx_data.motor4_position = rx_motor_position[2];
		pc_send_msg.tx_data.motor5_position = -rx_motor_position[3];
		pc_send_msg.tx_data.motor6_position = rx_motor_position[4]-3.14f;
		pc_send_msg.frame_tailer.tailer = 0x65;
		memcpy(PC_SEND_BUF, &pc_send_msg, sizeof(pc_send_msg));
		PC_SEND_BUF[LEN_TX_PACKET]= '\n';
		HAL_UART_Transmit(&huart2, PC_SEND_BUF, LEN_TX_PACKET + 1, 2000);
		osDelayUntil(&mode_wake_time, 7);
	}
}
uint8_t res = 0;
static void data_solve(receive_msg_t *pc_receive_msg, uint8_t *rx_data)
{
	if(rx_data[SOF_ADDR] == FRAME_HEADER && rx_data[LEN_RX_PACKET-1] == FRAME_TAILER) 
	{	
			memcpy(pc_receive_msg, rx_data, LEN_RX_PACKET);	
//			pc_receive_msg->rx_data.motor3_position = -pc_receive_msg->rx_data.motor3_position;
			pc_receive_msg->rx_data.motor1_position = -pc_receive_msg->rx_data.motor1_position;
			pc_receive_msg->rx_data.motor5_position = -pc_receive_msg->rx_data.motor5_position;
//			pc_receive_msg->rx_data.motor6_position += 3.14f;
	}
	else
	{
		
	}
}
//触发空闲中断时进行数据的校验与拷贝
void usart2_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart2.hdmarx); 
		USART2_RX_STA = USART2_MAX_RECV_LEN - huart2.hdmarx->Instance->NDTR; 
		__HAL_DMA_DISABLE(huart2.hdmarx);
		data_solve(&pc_receive_msg,USART2_RX_BUF);
		__HAL_DMA_CLEAR_FLAG(huart2.hdmarx, DMA_FLAGS);	
		__HAL_DMA_SET_COUNTER(huart2.hdmarx,USART2_MAX_RECV_LEN);
		__HAL_DMA_ENABLE(huart2.hdmarx);
	}
}





