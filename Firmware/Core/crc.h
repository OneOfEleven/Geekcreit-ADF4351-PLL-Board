
#ifndef crch
#define crch

#include <stdint.h>

#ifdef __cplusplus
	extern "C" {
#endif

void makeCRC16Table(void);
uint16_t updateCRC16(uint16_t crc, const void *data, const int len);

#ifdef __cplusplus
	}
#endif

#endif

