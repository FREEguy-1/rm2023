#ifndef A_COMMUNICATE_TASK_H
#define A_COMMUNICATE_TASK_H

#include "struct_typedef.h"

#define COMMUNICATE_HEADER  0xA5
#define COMMUNICATE_MIDDLE  0xA8
#define COMMUNICATE_TAIL 0xA3
#define COMMUNICATE_TX_LEN 19
#define COMMUNICATE_RX_LEN 17


extern fp32 rx_motor_position[5];  //当前电机反馈角度


extern void A_communicate_task(void const * argument);
extern void uart5_IRQHandler(void);

#endif

