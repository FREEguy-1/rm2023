#include "VOFA.h"
#include "chassis_task.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "pc_info_task.h"


float chassis_motor0_speed ,chassis_motor0_speed_set;
float chassis_motor1_speed ,chassis_motor1_speed_set;
float chassis_motor2_speed ,chassis_motor2_speed_set;
float chassis_motor3_speed ,chassis_motor3_speed_set;
float angle, angle_set;
float speed, speed_set;
void vofa_task(void const * argument)
{   
	while(1)
	{
//					angle = catch_move.angle;
//					angle_set = catch_move.angle_set;
//					speed = catch_move.rotate_data[0].speed;
//					speed_set = catch_move.rotate_data[0].speed_set;
		chassis_motor0_speed       =(float)(chassis_move.motor_chassis[0].speed);
		chassis_motor0_speed_set   =(float)(chassis_move.motor_chassis[0].speed_set);
		chassis_motor1_speed       =(float)(chassis_move.motor_chassis[1].speed);
		chassis_motor1_speed_set   =(float)(chassis_move.motor_chassis[1].speed_set);
		chassis_motor2_speed       =(float)(chassis_move.motor_chassis[2].speed);
		chassis_motor2_speed_set   =(float)(chassis_move.motor_chassis[2].speed_set);
		chassis_motor3_speed       =(float)(chassis_move.motor_chassis[3].speed);
		chassis_motor3_speed_set   =(float)(chassis_move.motor_chassis[3].speed_set);

//           printf("samples: %f,%f,%f,%f,%f,%f\n" , angle,angle_set,speed,speed_set,chassis_motor0_speed,chassis_motor0_speed_set );

		//底盘四个电机          
		printf("samples: %f,%f,%f,%f,%f,%f,%f,%f\n" , chassis_motor0_speed,chassis_motor0_speed_set,chassis_motor1_speed,chassis_motor1_speed_set,chassis_motor2_speed,chassis_motor2_speed_set,chassis_motor3_speed,chassis_motor3_speed_set );

		vTaskDelay(10);
	}
}
