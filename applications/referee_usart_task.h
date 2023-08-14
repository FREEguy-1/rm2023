#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

extern void referee_usart_task(void const * argument);
extern void USART6_IRQHandler(void);
#endif
