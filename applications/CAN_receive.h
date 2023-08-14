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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
} can_msg_id_3508_e;

typedef enum
{
	CAN_CATCH_ALL_ID = 0x1FF,
	CAN_ROTATEL_MOTOR_ID = 0x205,
	CAN_ROTATER_MOTOR_ID = 0x206,
	CAN_STRETCHL_MOTOR_ID = 0x207,
	CAN_STRETCHR_MOTOR_ID = 0x208,
}can_msg_id_2006_e;

//rm motor data
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          ���͵��̵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**
  * @brief          ����rotate���Ƶ���(0x205,0x206) stretch������Ƶ���(0x207,0x208)
  * @param[in]      rotatel: (0x205) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rotater: (0x206) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      stretchl: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      stretchr: (0x208) 2006������Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd_catch(int16_t rotatel, int16_t rotater, int16_t stretchl, int16_t stretchr);

/**
  * @brief          ����ץȡ���catch 2006�������ָ��
  * @param[in]      i: ������,��Χ[4,7]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_catch_motor_measure_point(uint8_t i);

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
