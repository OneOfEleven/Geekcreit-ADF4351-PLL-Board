
#include "common.h"

UART_HandleTypeDef    huart1 = {0};
#ifdef USE_IWDG
	IWDG_HandleTypeDef hiwdg  = {0};
#endif
CRC_HandleTypeDef     hcrc   = {0};

t_eeprom eeprom;

t_button left_button;
t_button right_button;
t_button up_button;
t_button down_button;
t_button ok_button;

void delay_ms(uint32_t ms)
{
	if (ms > 0)
	{
		const uint32_t tick = HAL_GetTick();
		while ((HAL_GetTick() - tick) < ms)
		{
			__WFI();		// wait here until an interrupt nudges us
		}
	}
}

void OPTIMIZE_SPEED delay_ns(uint32_t ns)
{
	#ifdef DWT
		ITM->LAR = 0xC5ACCE55;									// unlock the debug registers
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	//
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;				// turn on the cycle counter
		DWT->CYCCNT = 0;											// reset cycle counter
		volatile uint32_t cycles = ((uint64_t)HAL_RCC_GetHCLKFreq() * ns) / 1000000000;
		while (DWT->CYCCNT < cycles);
	#else
		volatile uint32_t cycles = ((uint64_t)HAL_RCC_GetHCLKFreq() * ns) / 1000000000;
		if (cycles > 0)
			while (--cycles != 0);
	#endif
}
