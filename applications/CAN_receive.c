/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
//#include "catch_task.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
		4:stretchļ����� 3508���; 5:stretchŗ����� 3508���; 6:rotatelץȡ��� 3508���; 7:rotaterץȡ��� 3508���*/
motor_measure_t motor_3508[4];
motor_measure_t motor_2006[4];

static CAN_TxHeaderTypeDef  catch_tx_message;
static uint8_t              catch_can_send_data[8];	
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		if(hcan == &CHASSIS_CAN)
		{	
				switch (rx_header.StdId)
				{
						case CAN_3508_M1_ID:
						case CAN_3508_M2_ID:
						case CAN_3508_M3_ID:
						case CAN_3508_M4_ID:
						{
								static uint8_t i = 0;
								//get motor id
								i = rx_header.StdId - CAN_3508_M1_ID;
								get_motor_measure(&motor_3508[i], rx_data);
								break;
						}

						default:
						{
								break;
						}
				}
		}
		else if(hcan == &GIMBAL_CAN)
		{
				switch (rx_header.StdId)
				{
						case CAN_ROTATEL_MOTOR_ID:
						case CAN_ROTATER_MOTOR_ID:
						case CAN_STRETCHL_MOTOR_ID:
						case CAN_STRETCHR_MOTOR_ID:

						{
//								static uint8_t i = 0;
//								//get motor id
//								i = rx_header.StdId - CAN_ROTATEL_MOTOR_ID;
//								get_motor_measure(&motor_2006[i], rx_data);
//								if(i == 2)
//								{
//									catch_position_calculation(&catch_move, 0);
//								}
//								else if(i == 3)
//								{
//									catch_position_calculation(&catch_move, 1);
//								}
//								break;
						}

						default:
						{
								break;
						}
				}		
		
		
		}
}




/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}



/**
  * @brief          ���͵��̵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          ����rotate���Ƶ���(0x205,0x206) stretch������Ƶ���(0x207,0x208)
  * @param[in]      rotatel: (0x205) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rotater: (0x206) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      stretchl: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      stretchr: (0x208) 2006������Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
void CAN_cmd_catch(int16_t rotatel, int16_t rotater, int16_t stretchl, int16_t stretchr)
{
    uint32_t send_mail_box;
    catch_tx_message.StdId = CAN_CATCH_ALL_ID;
    catch_tx_message.IDE = CAN_ID_STD;
    catch_tx_message.RTR = CAN_RTR_DATA;
    catch_tx_message.DLC = 0x08;
    catch_can_send_data[0] = (rotatel >> 8);
    catch_can_send_data[1] = rotatel;
    catch_can_send_data[2] = (rotater >> 8);
    catch_can_send_data[3] = rotater;
    catch_can_send_data[4] = (stretchl >> 8);
    catch_can_send_data[5] = stretchl;
    catch_can_send_data[6] = (stretchr >> 8);
    catch_can_send_data[7] = stretchr;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &catch_tx_message, catch_can_send_data, &send_mail_box);
}

/**
  * @brief          ����ץȡ���catch 2006�������ָ��
  * @param[in]      i: ������,��Χ[4,7]
  * @retval         �������ָ��
  */
const motor_measure_t *get_catch_motor_measure_point(uint8_t i)
{
    return &motor_2006[(i & 0x03)];
}


/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_3508[(i & 0x03)];
}

