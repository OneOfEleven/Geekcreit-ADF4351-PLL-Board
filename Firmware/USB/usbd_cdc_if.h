
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS_Ready(void);
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
	}
#endif

#endif
