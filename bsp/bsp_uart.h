#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_lpuart_edma.h"
#include "bsp_key.h"

void END_SEND(void);
void uart_Init(void);
void uart_send(uint8_t *uartdata);
void DEMO_LPUART_IRQHandler(void);
void LPUART_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);




#endif /* __BSP_UART_H */
