
#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "common.h"

#if (STM32_FLASH_SIZE < 256)
	#define STM_SECTOR_SIZE       1024
#else
	#define STM_SECTOR_SIZE       2048
#endif

void STMFLASH_Erase(void);
void STMFLASH_Read(const uint32_t offset_addr, uint16_t *pBuffer, const uint16_t length);
void STMFLASH_Write(uint32_t offset_addr, uint16_t *pBuffer, uint16_t length);

#ifdef __cplusplus
	}
#endif

#endif

















