#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"
#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 50
//�������Լ���Ӧ�豸˳��
enum errorList
{
		DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    GIMBAL_PIT_TOE,
    GIMBAL_YAW_TOE,
		ERROR_LIST_LENGHT,
};
typedef __packed struct
{
		uint64_t error_time;
    uint32_t new_time;
    uint16_t set_offline_time : 12;
    uint8_t error_exist : 1;
} error_t;
//�������
extern void detect_task(void const *pvParameters);
//��¼ʱ��
extern void detect_hook(uint8_t toe);
//��ȡ�����б�
extern const error_t *get_error_list_point(void);
#endif
