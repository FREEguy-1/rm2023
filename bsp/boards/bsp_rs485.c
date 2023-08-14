#include "detect_task.h"
#include "bsp_rs485.h"//电机通信协议
#include "arm_math.h"
#include "usart.h"
#include "dma.h"
#include "arm_zero.h"
#include "user_lib.h"
#include "arm_zero_task.h"

uint8_t RxBuffer_1[78];//HAL库使用的串口接收缓冲
uint8_t RxBuffer_2[78];//HAL库使用的串口接收缓冲
uint32_t CRC_32=0;
uint32_t message_1 = 0;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
//extern uint32_t leg_communication_is_ready[4];
uint32_t leg_count[4];


uint32_t crc32_core(uint32_t* ptr, uint32_t len)//计算CRC校验码，输入为：待校验数据指针，数据包含的4字节整形数量（余数舍去）
{
		uint32_t xbit = 0;
		uint32_t data = 0;
		uint32_t CRC32 = 0xFFFFFFFF;
		const uint32_t dwPolynomial = 0x04c11db7;
		for (uint32_t i = 0; i < len; i++)
		{
				xbit = ((unsigned int)1 << 31);
				data = ptr[i];
				for (uint32_t bits = 0; bits < 32; bits++)
				{
						if (CRC32 & 0x80000000)
						{
								CRC32 <<= 1;
								CRC32 ^= dwPolynomial;
						}
						else
								CRC32 <<= 1;
						if (data & xbit)
								CRC32 ^= dwPolynomial;
						
						xbit >>= 1;
				}
		}
		return CRC32;
}

void rs485_uart_init(void)
{
	//串口1
  //开启清空标志位
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	//开启空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  
	
//	//串口6
//	//开启清空标志位
//  __HAL_UART_CLEAR_IDLEFLAG(&huart7);
//	//开启空闲中断
//  __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);	
}

void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
	{
//			HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer_1, 78);
			HAL_UART_Receive_DMA(&huart6,(uint8_t *)RxBuffer_1,78);
			if(RxBuffer_1[2]==0)
			{ 	

					for(int i=77; i>=74; --i) 
					{ 
							CRC_32 <<= 8; 
							CRC_32 += RxBuffer_1[i]; 
					} 
					if(CRC_32==crc32_core((uint32_t *)RxBuffer_1,18))
					{
							memcpy(&arm_move.arm_motor.motor_recv.motor_recv_data,RxBuffer_1,78*sizeof(uint8_t));
							extract_data(&arm_move.arm_motor.motor_recv);						
					}
			}
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

		



uint8_t     MOTOR_SEND_BUF_0[34];
uint8_t     MOTOR_SEND_BUF_1[34];
uint8_t     MOTOR_SEND_BUF_2[34];
uint8_t     MOTOR_SEND_BUF_3[34];


//串口1发送函数
void modify_and_send_data_0(MOTOR_send * motor)
{
		if (motor == NULL)
    {
        return;
    }
	  //帧头赋值
		motor->motor_send_data.head.start[0] = 0xFE;
	  motor->motor_send_data.head.start[1] = 0xEE;
		motor->motor_send_data.head.motorID  = motor->id;
	
    //数据帧
		motor->motor_send_data.Mdata.mode = motor->mode;
		motor->motor_send_data.Mdata.ModifyBit = 0xFF;
	  motor->motor_send_data.Mdata.T    = motor->T*256;
	  motor->motor_send_data.Mdata.W    = motor->W*128.0f*REDUCTION_RATIO;	
		motor->motor_send_data.Mdata.Pos  = (int32_t)(motor->Pos*16384.0f/(2.0f*PI)*REDUCTION_RATIO);
	  motor->motor_send_data.Mdata.K_P  = motor->K_P*2048;
	  motor->motor_send_data.Mdata.K_W  = motor->K_W*1048;	
	  //计算CRC校验码
		motor->motor_send_data.CRCdata.u32= crc32_core((uint32_t *)motor,7);
	
		memcpy(MOTOR_SEND_BUF_0, &motor->motor_send_data, sizeof(motor->motor_send_data));	

	  HAL_UART_Transmit_DMA(&huart6, MOTOR_SEND_BUF_0, 34);
//		HAL_UART_Transmit(&huart1,MOTOR_SEND_BUF_0,34,2000);
}
//串口1发送函数

//将接收到的数据解读
void extract_data(MOTOR_recv * motor)   
{
		if (motor == NULL)
    {
        return;
    }
		motor->motor_id = motor->motor_recv_data.head.motorID;
		motor->mode     = motor->motor_recv_data.Mdata.mode;	
		motor->Temp		  = motor->motor_recv_data.Mdata.Temp/256.0f;
		if(motor->motor_id == 0)
		{
				motor->T = motor->motor_recv_data.Mdata.T/256.0f*REDUCTION_RATIO;
		}
		else if(motor->motor_id == 1)
		{
				motor->T = -motor->motor_recv_data.Mdata.T/256.0f*REDUCTION_RATIO;		
		}
		motor->W 				= motor->motor_recv_data.Mdata.W/128.0f/REDUCTION_RATIO;					
		motor->Acc 			= motor->motor_recv_data.Mdata.Acc/REDUCTION_RATIO;	
		motor->Pos		  = (float)(motor->motor_recv_data.Mdata.Pos)/16384.0f*(2*PI)/(9.1f);
			
//		if(fabs(motor->LW - motor->motor_recv_data.Mdata.LW/REDUCTION_RATIO) < 21.0f)
//		{
				motor->LW 	= motor->motor_recv_data.Mdata.LW;
		//}		
}




//  CriticalVelocity = 1.5, CriticalTorque = 0.01, StartupTorque = 0.05;
float A1_Friction_Compensation(MOTOR_recv *motor, float torque)
{
    if (fabsf(motor->motor_recv_data.Mdata.LW) > CriticalVelocity)
        torque += StartupTorque * sign(motor->motor_recv_data.Mdata.LW);
    else if (fabsf(torque) > CriticalTorque) 
        torque += StartupTorque * sign(torque);
    return torque;
}

