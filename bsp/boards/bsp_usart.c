#include "bsp_usart.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 
uint16_t USART2_RX_STA = 0;

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

//和算法通信的串口初始化
void usart2_idle_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	UART_Receive_DMA_No_IT(&huart2, USART2_RX_BUF , USART2_MAX_RECV_LEN);
}

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
  if (tmp1 == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
        return HAL_ERROR;
    }
    __HAL_LOCK(huart);
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->ErrorCode  = HAL_UART_ERROR_NONE;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)pData, Size);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UNLOCK(huart);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

//void usart7_init(void)
//{
//    //enable receive interrupt and idle interrupt
//    //使能接收中断和空闲中断
//    __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);  //receive interrupt
//    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);  //idle interrupt

//}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void uart8_init(void)
{
  //open uart idle it	
  //使能接收中断和空闲中断
  __HAL_UART_CLEAR_IDLEFLAG(&huart8);
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);  //receive interrupt
}

void uart7_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart7);
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);  //receive interrupt
}

void uart5_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

