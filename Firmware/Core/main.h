
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "common.h"

// *******************************************************

extern uint8_t dB_index;

void Error_Handler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void USART1_IRQHandler(void);
#ifdef USE_USB_VCP
	void USB_HP_CAN1_TX_IRQHandler(void);
	void USB_LP_CAN1_RX0_IRQHandler(void);
	void CDC_Receive_CB(uint8_t *Buf, uint32_t Len);
#endif

//int __io_putchar(int ch);
int _write(int file, char *ptr, int len);

// *******************************************************

#ifdef __cplusplus
	}
#endif

#endif
