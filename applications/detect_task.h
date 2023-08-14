#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"
#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 50
//错误码以及对应设备顺序
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
//检测任务
extern void detect_task(void const *pvParameters);
//记录时间
extern void detect_hook(uint8_t toe);
//获取错误列表
extern const error_t *get_error_list_point(void);
#endif
