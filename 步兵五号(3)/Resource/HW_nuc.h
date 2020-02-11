#ifndef _NUC_H_
#define _NUC_H_

#include "stdint.h"
#include "usart.h"
#include "math.h"
#include "mode_update.h"

#define NUC_UART_HANDLER  huart3
#define NUC_UART_RxHEAD1  0xAA
#define NUC_UART_RxHEAD2  0xBB
#define NUC_UART_RxHEAD3  0xCC
#define NUC_RxLEN         9         //接收帧头3个，有效字节6个
#ifndef PI
#define PI 3.1415926535897932384626f               //正式使用时请注释
#endif

extern uint8_t NucRxBuf[9];
extern short temp1, temp2, temp3, temp4;
extern short pre_temp3;
extern uint8_t isNUC;
extern uint32_t nuc_tick;
void NUC_Init(void);
void NUC_USART_IRQHandler(void);
void NUC_Receive_Check(uint8_t data);
void NUC_Receive_Process(uint8_t * Buf);
#endif
