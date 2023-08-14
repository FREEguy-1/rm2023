#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"

//最大缓冲字节数
#define USART2_MAX_RECV_LEN 100
//接收缓冲
extern uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];
//当前缓冲长度
extern uint16_t USART2_RX_STA;

extern void usart2_idle_init(void);
extern void uart7_init(void);
extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void uart8_init(void);
extern void uart5_init(void);

#endif
