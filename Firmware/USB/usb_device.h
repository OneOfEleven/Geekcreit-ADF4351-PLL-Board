
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"

/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
	}
#endif

#endif
