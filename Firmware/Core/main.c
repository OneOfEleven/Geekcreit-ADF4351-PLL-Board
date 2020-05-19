
// 16th May 2020, G6AMU

#include <stdio.h>		// printf
#include <string.h>		// memset, memcpy
#include <stdlib.h>		// malloc, free, rand

#include "main.h"
#include "oled.h"
#include "crc.h"
#include "stmflash.h"

#ifdef USE_USB_VCP
	#include "usb_device.h"
	#include "usbd_cdc_if.h"
#endif

#include "adf4351.h"

// *********************************************************************

#define CONNECTION_TIMEOUT_MS       20000		//
#define UART_RX_TIMEOUT_MS          50			// UART Rx packet data timeout

#define CHAR_WIDTH                  8
#define LEADING_ZERO_CHAR           '_'

#define FREQ_MODE_SPOT              0
#define FREQ_MODE_SWEEP             1
#define FREQ_MODE_HOP               2

// *********************************************************************
// selected menu option

#define TASK_NONE                       -1

#define TASK_FREQ                        0
#define TASK_SWEEP_FREQ                  1
#define TASK_HOP_FREQ                    2
#define TASK_SCAN_STEP_FREQ              3
#define TASK_SCAN_SPEED                  4
#define TASK_OUTPUT_POWER                5
#define TASK_REF_FREQ                    6
#define TASK_REF_FREQ_ADJUST             7
#define TASK_CHAN_SPACE                  8
#define TASK_CHG_PUMP_CUR                9
#define TASK_UNLOCK_MUTE                10
#define TASK_MAIN_MENU                  11

// *********************************************************************

// serial RX buffer structure
typedef struct t_rx_buffer
{
	uint8_t __attribute__ ((aligned(4))) buffer[512];
	volatile unsigned int buffer_wr;
	volatile unsigned int timer;
} t_rx_buffer;

// *********************************************************************

const uint16_t version = 0x0100;

// firmware title
const char title[] __attribute__ ((section(".my_flash_data"))) __attribute__ ((used)) __attribute__ ((aligned(2))) =
		"STM32F103-ADF4351-OLED, 16th May 2020";

#ifdef USE_USB_VCP
	t_rx_buffer usb_rx = {0};
#endif
t_rx_buffer uart1_rx  = {0};

#ifdef USE_IWDG
	volatile uint16_t iwdg_reload_tick = 0xffff;
#endif

/*
struct cpu_details
{
	uint32_t cpuID;
	uint32_t sysclock;
	uint32_t hclock;
	uint32_t pclock1;
	uint32_t pclock2;
	uint32_t deviceID;
	uint32_t revisionID;
	uint32_t flashSize_bytes;
	uint32_t uniqueID_96[3];
	uint8_t  bootloader_id;
	uint32_t package_type;
} cpu_details;
*/

typedef struct
{
		uint32_t freq_in_Hz;
		bool     mul2_enable;
		uint16_t r_divider;
		bool     div2_enable;
} t_ref_freq;

// these params are to ensure that the PFD frequency is always 10 MHz
const t_ref_freq ref_freq_table[] =
{
		{  5000000, true,   1, false},
		{ 10000000, true,   1, true},
		{ 15000000, true,   3, false},
		{ 20000000, false,  1, true},
		{ 25000000, true,   5, false},
		{ 30000000, true,   3, true},
		{ 35000000, true,   7, false},
		{ 40000000, false,  2, true},
		{ 45000000, true,   9, false},
		{ 50000000, true,   5, true},
		{ 55000000, true,  11, false},
		{ 60000000, false,  3, true},
		{ 65000000, true,  13, false},
		{ 70000000, true,   7, true},
		{ 75000000, true,  15, false},
		{ 80000000, false,  4, true},
		{ 85000000, true,  17, false},
		{ 90000000, true,   9, true},
		{ 95000000, true,  19, false},
		{100000000, false,  5, true},
		{105000000, true,  21, false},
		{110000000, true,  11, true},
		{115000000, true,  23, false},
		{120000000, false,  6, true},
		{125000000, true,  25, false},
		{130000000, false, 13, false},
		{140000000, false,  7, true},
		{150000000, false, 15, false},
		{160000000, false,  8, true},
		{170000000, false, 17, false},
		{180000000, false,  9, true},
		{190000000, false, 19, false},
		{200000000, false, 10, true},
		{210000000, false, 21, false},
		{220000000, false, 11, true},
		{230000000, false, 23, false},
		{240000000, false, 12, true},
		{250000000, false, 25, false}
};

// uA
const uint16_t charge_pump_current_table[] = {313, 625, 938, 1250, 1560, 1880, 2190, 2500, 2810, 3130, 3440, 3750, 4060, 4380, 4690, 5000};

#ifdef USE_USB_VCP
	uint8_t *usb_tx_buf    = nullptr;                  //
	size_t usb_tx_buf_size = 0;                        //
#endif

uint8_t *uart_tx_buf      = nullptr;                  //
size_t uart_tx_buf_size   = 0;                        //

t_adf4351_dev adf4351_dev;                            // holds the current params and start for the ADF4351 chip

volatile uint16_t sweep_hop_count = 0;                // frequency step time counter
volatile uint64_t sweep_hop_freq  = 0;                // the frequency we are currently on
volatile bool     sweep_hop_next  = false;            // interrupt sets this to tell the EXEC when to step to the next frequency

int task_index            = 0;                        // current selected menu option
int task_id               = TASK_NONE;                // current menu process being ran

volatile uint32_t random32;

// *********************************************************************

void reboot(void)
{
	__disable_irq();
	HAL_NVIC_SystemReset();
	while (1);
}

// *********************************************************************

void button_debouce_reset(t_button *button)
{
	if (button == nullptr)
		return;
	button->debounce_tick    = 0;
	button->long_press_tick  = 0;
	button->down             = false;
	button->pressed          = false;
	button->pressed_long     = false;
	button->released         = false;
}

void button_debouce_set(t_button *button, bool pressed)
{
	if (button == nullptr)
		return;

	if (pressed)
	{	// button is pressed
		button->debounce_tick = BUTTON_DEBOUNCE_MS;
		button->down          = true;
	}
	else
	{	// no it's not
		button->debounce_tick = 0;
		button->down          = false;
	}

	button->long_press_tick  = 0;
	button->pressed          = false;
	button->pressed_long     = false;
	button->released         = false;
}

void button_debouce(t_button *button, bool pressed)
{
	if (button == nullptr)
		return;

	if (pressed)
	{	// button is pressed
		if (button->debounce_tick < BUTTON_DEBOUNCE_MS)
		{
			if (++button->debounce_tick >= BUTTON_DEBOUNCE_MS && !button->down)
			{
				button->down    = true;
				button->pressed = true;
			}
		}
		if (button->long_press_tick < BUTTON_DEBOUNCE_LONG_MS)
		{
			if (++button->long_press_tick >= BUTTON_DEBOUNCE_LONG_MS && !button->down_long)
			{
				button->down_long    = true;
				button->pressed_long = true;
			}
		}
	}
	else
	{	// no it's not
		if (button->debounce_tick > 0)
		{
			if (--button->debounce_tick <= 0 && button->down)
			{
				button->down            = false;
				button->down_long       = false;
				button->released        = true;
				button->long_press_tick = 0;
			}
		}
		if (button->long_press_tick > 0)
			button->long_press_tick--;
	}
}

void process_but_releases(void)
{
	if (left_button.released)  button_debouce_reset(&left_button);
	if (right_button.released) button_debouce_reset(&right_button);
	if (up_button.released)    button_debouce_reset(&up_button);
	if (down_button.released)  button_debouce_reset(&down_button);
	if (ok_button.released)    button_debouce_reset(&ok_button);
}

// *********************************************************************

void Error_Handler(void)
{
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
	while (1)
	{
	}
}

void MemManage_Handler(void)
{
	while (1)
	{
	}
}

void BusFault_Handler(void)
{
	while (1)
	{
	}
}

void UsageFault_Handler(void)
{
	while (1)
	{
	}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
	HAL_IncTick();

	#ifdef USE_USB_VCP
		if (usb_rx.timer < 0xffff)
			usb_rx.timer++;
	#endif

	if (uart1_rx.timer < 0xffff)
		uart1_rx.timer++;

	#ifdef USE_IWDG
		if (iwdg_reload_tick < 0xffff)
			iwdg_reload_tick++;
	#endif

	button_debouce(&left_button,  (HAL_GPIO_ReadPin(BUT_LEFT_GPIO_Port,  BUT_LEFT_Pin)  == GPIO_PIN_RESET) ? true : false);
	button_debouce(&right_button, (HAL_GPIO_ReadPin(BUT_RIGHT_GPIO_Port, BUT_RIGHT_Pin) == GPIO_PIN_RESET) ? true : false);
	button_debouce(&up_button,    (HAL_GPIO_ReadPin(BUT_UP_GPIO_Port,    BUT_UP_Pin)    == GPIO_PIN_RESET) ? true : false);
	button_debouce(&down_button,  (HAL_GPIO_ReadPin(BUT_DOWN_GPIO_Port,  BUT_DOWN_Pin)  == GPIO_PIN_RESET) ? true : false);
	button_debouce(&ok_button,    (HAL_GPIO_ReadPin(BUT_OK_GPIO_Port,    BUT_OK_Pin)    == GPIO_PIN_RESET) ? true : false);

	switch (eeprom.mode)
	{
		case FREQ_MODE_SPOT:
			break;

		case FREQ_MODE_SWEEP:
		case FREQ_MODE_HOP:
			if (!sweep_hop_next)
			{
				if (++sweep_hop_count >= eeprom.scan_timer_ms)
				{
					sweep_hop_count = 0;
					sweep_hop_next = true;	// tell the exec to move too the next frequency
				}
			}
			break;
	}
}

void USART1_IRQHandler(void)
{
	//	HAL_UART_IRQHandler(&huart1);

	UART_HandleTypeDef *huart = &huart1;

	uint32_t isrflags   = READ_REG(huart->Instance->SR);
	uint32_t cr1its     = READ_REG(huart->Instance->CR1);
	//uint32_t cr3its     = READ_REG(huart->Instance->CR3);

	uint32_t errorflags = isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE);

	// *************************
	// receiver

	if (errorflags)	// errors
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_PE | UART_FLAG_FE);

	//if (!errorflags)
	{
		if (isrflags & USART_SR_RXNE)
		{
			// fetch the Rx'ed byte
			const uint8_t data = (uint8_t)READ_REG(huart->Instance->DR);

			if (cr1its & USART_CR1_RXNEIE)
			{	// interrupt is enabled

				t_rx_buffer *uart_rx = nullptr;
				if (huart->Instance == USART1) uart_rx = &uart1_rx;
				//else
				//if (huart->Instance == USART2) uart_rx = &uart2_rx;

				if (uart_rx != nullptr)
				{
					unsigned int wr = uart_rx->buffer_wr;

					// save the rx'ed byte into our RX circular buffer
					//uart_rx->buffer[wr] = data;
					//if (++wr >= ARRAY_SIZE(uart_rx->buffer))
					//	wr = 0;

					// save the rx'ed byte into our non-circular RX buffer
					if (wr < ARRAY_SIZE(uart_rx->buffer))
						uart_rx->buffer[wr++] = data;

					uart_rx->buffer_wr = wr;

					// reset the RX timer
					uart_rx->timer = 0;
				}
			}
		}
	}

	// *************************
	// transmitter

	if ((isrflags & USART_SR_TXE) && (cr1its & USART_CR1_TXEIE))
	{	// TX is ready for another byte
		bool finish = true;

		if (huart->pTxBuffPtr && huart->TxXferCount > 0 && huart->gState == HAL_UART_STATE_BUSY_TX)
		{	// send next byte
			huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++);
			if (--huart->TxXferCount > 0)
				finish = false;
		}

		if (finish)
		{
			__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);		// Disable the UART Transmit Complete Interrupt
			huart->TxXferCount = 0;
			huart->pTxBuffPtr  = nullptr;
			huart->gState      = HAL_UART_STATE_READY;
		}
	}

	// transmission end
	if ((isrflags & USART_SR_TC) && (cr1its & USART_CR1_TCIE))
	{
		__HAL_UART_DISABLE_IT(huart, UART_IT_TC);				// Disable the UART Transmit Complete Interrupt
		huart->TxXferCount = 0;
		huart->pTxBuffPtr  = nullptr;
		huart->gState      = HAL_UART_STATE_READY;
	}

	// *************************
}

#ifdef USE_USB_VCP
	void USB_HP_CAN1_TX_IRQHandler(void)
	{
		HAL_PCD_IRQHandler(&hpcd_USB_FS);
	}

	void USB_LP_CAN1_RX0_IRQHandler(void)
	{
		HAL_PCD_IRQHandler(&hpcd_USB_FS);
	}

	void CDC_Receive_CB(uint8_t *Buf, uint32_t Len)
	{
		unsigned int wr = usb_rx.buffer_wr;

		if (Len > (ARRAY_SIZE(usb_rx.buffer) - wr))
			Len = ARRAY_SIZE(usb_rx.buffer) - wr;

		memcpy(&usb_rx.buffer[wr], Buf, Len);
		wr += Len;

		usb_rx.buffer_wr = wr;
		usb_rx.timer = 0;
	}
#endif

// *********************************************************************

void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

//	#ifdef _DEBUG
		// NOJTAG: JTAG-DP Disabled and SW-DP Enabled
		__HAL_AFIO_REMAP_SWJ_NOJTAG();
//	#else
		// NOJTAG: JTAG-DP Disabled and SW-DP Disabled ...
		// careful !! .. if this option is used then you'll need to pull the BOOT-0 CPU pin high at reset to reprogram the CPU
//		__HAL_AFIO_REMAP_SWJ_DISABLE();
//	#endif
}

// *********************************************************************

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

// *********************************************************************

#ifdef USE_IWDG
	void MX_IWDG_Init(void)
	{
		memset(&hiwdg, 0, sizeof(hiwdg));

		hiwdg.Instance       = IWDG;

//		hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
//		hiwdg.Init.Reload    = 4095;

		// IWDG counter clock Frequency = uwLsiFreq
		// Prescaler to 256
		// Timeout Period = (Reload Counter Value * 256) / LSI_VALUE
		//
		// Reload value for 5 second IWDG TimeOut
		// So Set Reload Counter Value = (LSI_VALUE * 5) / 256
		hiwdg.Init.Prescaler = IWDG_PRESCALER_256;	// 4, 8, 16, 32, 64, 128 or 256
		hiwdg.Init.Reload    = (LSI_VALUE * 5) / 256;

		if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
		{
			hiwdg.Instance = nullptr;
			return;
		}

		iwdg_reload_tick = 0;
	}

	void service_IWDG(bool force)
	{	// service the watchdog
		if (hiwdg.Instance != nullptr && (iwdg_reload_tick >= 4000 || force))
		{
			if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK)
			{
				iwdg_reload_tick = 4000;
//				printf("IWDG: has been fed\r\n");
			}
//			else
//				printf("IWDG: has not been fed\r\n");
		}
	}
#endif

// *********************************************************************

void MX_USART1_UART_Init(void)
{
	memset(&huart1, 0, sizeof(huart1));

	huart1.Instance          = USART1;
	huart1.Init.BaudRate     = SERIAL_BAUDRATE;
	huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	huart1.Init.StopBits     = UART_STOPBITS_1;
	huart1.Init.Parity       = UART_PARITY_NONE;
	huart1.Init.Mode         = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		huart1.Instance = nullptr;
		return;
	}

	__HAL_UART_FLUSH_DRREGISTER(&huart1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (huart->Instance == USART1)
	{
		// Peripheral clock enable
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Pin   = UART_TX_Pin;
		HAL_GPIO_Init(UART_TX_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Pin  = UART_RX_Pin;
		HAL_GPIO_Init(UART_RX_GPIO_Port, &GPIO_InitStruct);

		__HAL_UART_ENABLE_IT(huart, UART_IT_PE);
		// Enable the UART Error Interrupt: (Frame error, noise error, overrun error)
		__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
		// Enable the UART Data Register not empty Interrupt
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

		// USART1 interrupt Init
		HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// Peripheral clock disable
		__HAL_RCC_USART1_CLK_DISABLE();

		HAL_GPIO_DeInit(UART_TX_GPIO_Port, UART_TX_Pin);
		HAL_GPIO_DeInit(UART_RX_GPIO_Port, UART_RX_Pin);

		// USART1 interrupt DeInit
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
}

// *********************************************************************

void MX_CRC_Init(void)
{
	memset(&hcrc, 0, sizeof(hcrc));

	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		hcrc.Instance = nullptr;
		return;
	}
}

void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
	if (hcrc->Instance == CRC)
	{
		// Peripheral clock enable
		__HAL_RCC_CRC_CLK_ENABLE();
	}
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
	if (hcrc->Instance == CRC)
	{
		// Peripheral clock disable
		__HAL_RCC_CRC_CLK_DISABLE();
	}
}

// *********************************************************************

void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_GPIO_WritePin(LED1_GPIO_Port,  LED1_Pin,  GPIO_PIN_RESET);

	HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(ADF4351_CE_GPIO_Port,   ADF4351_CE_Pin,   GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF4351_DATA_GPIO_Port, ADF4351_DATA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADF4351_CLK_GPIO_Port,  ADF4351_CLK_Pin,  GPIO_PIN_RESET);

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = LED1_Pin;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = OLED_DC_Pin;
	HAL_GPIO_Init(OLED_DC_GPIO_Port,  &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = OLED_RST_Pin;
	HAL_GPIO_Init(OLED_RST_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = OLED_CLK_Pin;
	HAL_GPIO_Init(OLED_CLK_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = OLED_SDA_Pin;
	HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Pin   = BUT_LEFT_Pin;
	HAL_GPIO_Init(BUT_LEFT_GPIO_Port,  &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = BUT_OK_Pin;
	HAL_GPIO_Init(BUT_OK_GPIO_Port,    &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = BUT_DOWN_Pin;
	HAL_GPIO_Init(BUT_DOWN_GPIO_Port,  &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = BUT_RIGHT_Pin;
	HAL_GPIO_Init(BUT_RIGHT_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = BUT_UP_Pin;
	HAL_GPIO_Init(BUT_UP_GPIO_Port,    &GPIO_InitStruct);

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Pin   = ADF4351_CE_Pin;
	HAL_GPIO_Init(ADF4351_CE_GPIO_Port,   &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = ADF4351_LE_Pin;
	HAL_GPIO_Init(ADF4351_LE_GPIO_Port,   &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = ADF4351_DATA_Pin;
	HAL_GPIO_Init(ADF4351_DATA_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = ADF4351_CLK_Pin;
	HAL_GPIO_Init(ADF4351_CLK_GPIO_Port,  &GPIO_InitStruct);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin  = ADF4351_LD_Pin;
	HAL_GPIO_Init(ADF4351_LD_GPIO_Port, &GPIO_InitStruct);
}

// *********************************************************************

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif

// *********************************************************************

void send_serial(void *buf, size_t len, bool both)
{
	const uint32_t tick = HAL_GetTick();
	PCD_HandleTypeDef *hpcd_USB = nullptr;

	if (buf == 0 || len == 0)
		return;

	#ifdef USE_USB_VCP
		// send the data through the USB virtual communications port
		hpcd_USB = &hpcd_USB_FS;
		if (hpcd_USB->Instance != nullptr)
		{
			while ((HAL_GetTick() - tick) < 20)		// wait for up to 50ms
			{
				if (CDC_Transmit_FS_Ready() == USBD_OK)
				{	// usb transmit chain is ready for use

					if (usb_tx_buf == nullptr || usb_tx_buf_size < len)
					{	// create the required buffer
						if (usb_tx_buf != nullptr)
							free(usb_tx_buf);
						usb_tx_buf_size = ((len + 128) / 128) * 128;		// multiple of 128 bytes in size
						usb_tx_buf      = (uint8_t *)malloc(usb_tx_buf_size);
					}

					if (usb_tx_buf != nullptr)
					{	// copying into our own buffer frees up the printf buffer
						memcpy(usb_tx_buf, buf, len);
						CDC_Transmit_FS(usb_tx_buf, len);
					}
					else
						CDC_Transmit_FS((uint8_t *)buf, len);

					break;
				}
			}
		}
		else
			hpcd_USB = nullptr;
	#endif

	// send the data via the hardware serial port
	if (hpcd_USB == nullptr || both)
	{
		UART_HandleTypeDef *huart = &huart1;
		if (huart->Instance != nullptr)
		{
			while ((HAL_GetTick() - tick) < 20)		// wait for up to 50ms
			{
				if (huart->gState == HAL_UART_STATE_READY && huart->pTxBuffPtr == nullptr)
				{	// uart transmit chain is ready for use

					if (uart_tx_buf == nullptr || uart_tx_buf_size < len)
					{	// create the required buffer
						if (uart_tx_buf != nullptr)
							free(uart_tx_buf);
						uart_tx_buf_size = ((len + 128) / 128) * 128;		// multiple of 128 bytes in size
						uart_tx_buf      = (uint8_t *)malloc(uart_tx_buf_size);
					}

					if (uart_tx_buf != nullptr)
					{	// copying into our own buffer frees up the printf buffer
						memcpy(uart_tx_buf, buf, len);
						HAL_UART_Transmit_IT(huart, uart_tx_buf, len);
					}
					else
						HAL_UART_Transmit(huart, (uint8_t *)buf, len, 20);

					break;
				}
			}
		}
	}
}

// printf() intercept
int _write(int file, char *ptr, int len)
{
	// stdout .. file = 1
	// stderr .. file = 2
	if ((file == 1 || file == 2) && ptr && len > 0)
		send_serial(ptr, len, true);

	return len;
}

// *********************************************************************

void eeprom_save(void)
{
	// find the first unused (free) block in the 'emulated eeprom' flash sector to save our eeprom data in to - wear leveling
	uint32_t addr_last_valid = 0xffffffff;
	uint32_t addr = 0;
	while (addr <= (STM_SECTOR_SIZE - sizeof(t_eeprom)))
	{
		t_eeprom _eeprom;
		unsigned int i;
		const uint8_t *p = (uint8_t *)&_eeprom;

		STMFLASH_Read(addr, (uint16_t *)p, sizeof(t_eeprom) / sizeof(uint16_t));

		for (i = 0; i < sizeof(t_eeprom) && *p == 0xff; i++, p++);

		if (i >= sizeof(t_eeprom))
			break;                    // found an unused (free) block

		// TODO: crc

		if (_eeprom.magic == EEPROM_MAGIC && _eeprom.crc == 0xffff)
			addr_last_valid = addr;   // found a valid block

		addr += sizeof(t_eeprom);
	}

	if (addr >= (STM_SECTOR_SIZE - sizeof(t_eeprom)))
	{	// no unused blocks found .. erase the whole flash sector to start again
		addr = 0;
		addr_last_valid = 0xffffffff;

		#ifdef USE_IWDG
			service_IWDG(true);
		#endif

		STMFLASH_Erase();
	}

	// check to see if the already stored settings are the same as the current settings
	if (addr_last_valid != 0xffffffff)
	{
		t_eeprom _eeprom;
		STMFLASH_Read(addr_last_valid, (uint16_t *)&_eeprom, sizeof(t_eeprom) / sizeof(uint16_t));

		// TODO: crc

		if (memcmp(&_eeprom, &eeprom, sizeof(t_eeprom)) == 0)
			return;		// no need to save the settings as their are no differences
	}

	eeprom.magic      = EEPROM_MAGIC;
	//eeprom.padding[0] = 0xff;
	//eeprom.padding[1] = 0xff;
	eeprom.crc        = 0;
	//eeprom.crc      = updateCRC16(0xffff, &eeprom, sizeof(eeprom) - sizeof(eeprom.crc));    // TODO: crc
	eeprom.crc        = 0xffff;

	#ifdef USE_IWDG
		service_IWDG(true);
	#endif

	STMFLASH_Write(addr, (uint16_t *)&eeprom, sizeof(t_eeprom) / sizeof(uint16_t));
}

void eeprom_load(void)
{
	// find the last valid block that we saved in the 'emulated eeprom' flash sector - wear leveling
	uint32_t addr_last_valid = 0xffffffff;
	uint32_t addr = 0;
	while (addr <= (STM_SECTOR_SIZE - sizeof(t_eeprom)))
	{
		t_eeprom _eeprom;

		STMFLASH_Read(addr, (uint16_t *)&_eeprom, sizeof(t_eeprom) / sizeof(uint16_t));

		// TODO: crc

		if (_eeprom.magic == EEPROM_MAGIC && _eeprom.crc == 0xffff)
			addr_last_valid = addr;	// found a valid block

		addr += sizeof(t_eeprom);
	}
	if (addr_last_valid != 0xffffffff)
	{	// use the last valid block we found in the flash sector
		STMFLASH_Read(addr_last_valid, (uint16_t *)&eeprom, sizeof(t_eeprom) / sizeof(uint16_t));
	}
}

// x^y
uint64_t i_pow(uint8_t x, uint8_t y)
{
	uint64_t result = 1;
	while (y > 0)
	{
		y--;
		result *= x;
	}
	return result;
}

void uint_to_buf(uint64_t value, uint8_t buf[], const unsigned int buf_size)
{
	unsigned int i;
	uint64_t div = i_pow(10, buf_size - 1);
	for (i = 0; i < buf_size && div > 0; i++)
	{
		buf[i + 0] = value / div;
		buf[i + 1] = 0;
		value %= div;
		div /= 10;
	}
}

uint64_t buf_to_uint(const uint8_t buf[], const unsigned int buf_size)
{
	unsigned int i;
	uint64_t value = 0;
	uint64_t mul = i_pow(10, buf_size - 1);
	for (i = 0; i < buf_size && mul > 0; i++)
	{
		value += (uint64_t)buf[i] * mul;
		mul /= 10;
	}
	return value;
}

void copy_freq_buf_2_display(uint8_t source[], char dis[], const uint8_t dis_point, const char leading_zero_char)
{
	uint8_t leading_zero = 1;

	if (source[0] > 0 || !leading_zero) {dis[0] = source[0] + '0'; leading_zero = 0;} else dis[0] = leading_zero_char;
	if (source[1] > 0 || !leading_zero) {dis[1] = source[1] + '0'; leading_zero = 0;} else dis[1] = leading_zero_char;
	if (source[2] > 0 || !leading_zero) {dis[2] = source[2] + '0'; leading_zero = 0;} else dis[2] = leading_zero_char;
	dis[ 3] = source[3] + '0';
	dis[ 4] = '.';
	dis[ 5] = source[4] + '0';
	dis[ 6] = source[5] + '0';
	dis[ 7] = source[6] + '0';
	dis[ 8] = ' ';
	dis[ 9] = source[7] + '0';
	dis[10] = source[8] + '0';
	dis[11] = source[9] + '0';
	dis[12] = '\0';

	if (dis_point <= 3)
		dis[dis_point + 0] += 128;
	else
	if (dis_point >= 4 && dis_point <= 6)
		dis[dis_point + 1] += 128;
	else
	if (dis_point >= 7 && dis_point <= 9)
		dis[dis_point + 2] += 128;
}

void copy_time_buf_2_display(uint8_t source[], char dis[], uint8_t dis_point)
{
	uint8_t leading_zero = 1;

	if (source[0] > 0 || !leading_zero) {dis[0] = source[0] + '0'; leading_zero = 0;} else dis[0] = LEADING_ZERO_CHAR;
	if (source[1] > 0 || !leading_zero) {dis[1] = source[1] + '0'; leading_zero = 0;} else dis[1] = LEADING_ZERO_CHAR;
	if (source[2] > 0 || !leading_zero) {dis[2] = source[2] + '0'; leading_zero = 0;} else dis[2] = LEADING_ZERO_CHAR;
	dis[3] = source[3] + '0';
	dis[4] = 'm';
	dis[5] = 's';
	dis[6] = '\0';

	dis[dis_point] += 128;
}

void copy_current_buf_2_display(uint8_t source[], char dis[])
{
	dis[0] = source[0] + '0';
	dis[1] = '.';
	dis[2] = source[1] + '0';
	dis[3] = source[2] + '0';
	dis[4] = source[3] + '0';
	dis[5] = 'm';
	dis[6] = 'A';
	dis[7] = '\0';
}

void show_menu_frequency(void)
{
	unsigned int i;
	uint64_t div;
	uint8_t buf[10];
	char display[13];
	uint64_t freq = eeprom.frequency;

	div = i_pow(10, sizeof(buf) - 1);
	for (i = 0; i < sizeof(buf) && div > 0; i++)
	{
		buf[i + 0] = freq / div;
		buf[i + 1] = 0;
		freq %= div;
		div /= 10;
	}

	copy_freq_buf_2_display(buf, display, 255, ' ');
	LCD_Show_ModeCEStr(0, 6, " ", 1);
	OLED_ShowString(CHAR_WIDTH * 1, 6, display);
	LCD_Show_ModeCEStr(CHAR_WIDTH * 13, 6, "  ", 1);
}

void lcd_show_mode(void)
{
	switch (eeprom.mode)
	{
		case FREQ_MODE_SPOT:
			if (task_id == TASK_MAIN_MENU)
				show_menu_frequency();
			break;
		case FREQ_MODE_SWEEP:
			LCD_Show_ModeCEStr(0, 6, "*   SWEEPING   *", 1);
			break;
		case FREQ_MODE_HOP:
			LCD_Show_ModeCEStr(0, 6, "*   HOPPING    *", 1);
			break;
		default:
			LCD_Show_ModeCEStr(0, 6, "                ", 1);
			break;
	}
}

void inc_buf(uint8_t buf[], const unsigned int buf_size, int index)
{
	if (buf != NULL && index >= 0 && index < (int)buf_size && buf_size > 0)
	{
		if (++buf[index] > 9)
		{
			//if (index >= (int)(buf_size - 1))
			if (index <= 0)
				buf[index] = 9;
			else
			{
				buf[index--] = 0;
				inc_buf(buf, buf_size, index);
			}
		}
	}
}

void dec_buf(uint8_t buf[], const unsigned int buf_size, int index)
{
	if (buf != NULL && index >= 0 && index < (int)buf_size && buf_size > 0)
	{
		if (--buf[index] > 9)
		{
			//if (index >= (int)(buf_size - 1))
			if (index <= 0)
				buf[index] = 0;
			else
			{
				buf[index--] = 9;
				dec_buf(buf, buf_size, index);
			}
		}
	}
}

// spot frequency setting
void freq_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10];
	char display[13];

	uint_to_buf(eeprom.frequency, buf, sizeof(buf));

	if (task_id != TASK_FREQ)
	{
		task_id = TASK_FREQ;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Spot Frequency");
		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		if (ok_button.pressed_long)
		{
			if (eeprom.mode != FREQ_MODE_SPOT)
			{	// stop sweeping
				eeprom.mode = FREQ_MODE_SPOT;
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				lcd_show_mode();
			}
		}
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		inc_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		dec_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint64_t freq = buf_to_uint(buf, sizeof(buf));

			if (freq < ADF4351_OUT_MIN_HZ)
				freq = ADF4351_OUT_MIN_HZ;
			else
			if (freq > ADF4351_OUT_MAX_HZ)
				freq = ADF4351_OUT_MAX_HZ;

			if (eeprom.mode != FREQ_MODE_SPOT)
			{	// stop sweeping
				eeprom.mode = FREQ_MODE_SPOT;
				lcd_show_mode();
			}

			eeprom.frequency = freq;

			adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
		}

		uint_to_buf(eeprom.frequency, buf, sizeof(buf));

		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);

		process_but_releases();
	}
}

// frequency sweep start and end frequency settings
void sweep_freq_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10 * 2];
	char display1[13];
	char display2[13];

	uint_to_buf(eeprom.start_freq, &buf[              0], sizeof(buf) / 2);
	uint_to_buf(eeprom.end_freq,   &buf[sizeof(buf) / 2], sizeof(buf) / 2);

	if (task_id != TASK_SWEEP_FREQ)
	{
		task_id = TASK_SWEEP_FREQ;

		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Sweep Limits");

		copy_freq_buf_2_display(&buf[                0], display1, p_index - 0, LEADING_ZERO_CHAR);
		OLED_ShowString(0, 2, (char *)"fr ");
		OLED_ShowString(CHAR_WIDTH * 3, 2, display1);

		copy_freq_buf_2_display(&buf[(sizeof(buf) / 2)], display2, p_index - (sizeof(buf) / 2), LEADING_ZERO_CHAR);
		OLED_ShowString(0, 4, (char *)"to ");
		OLED_ShowString(CHAR_WIDTH * 3, 4, display2);

		lcd_show_mode();
	}

	if (ok_button.released)
	{
		if (eeprom.end_freq <= eeprom.start_freq)
		{
			OLED_ShowString(0, 6, (char *)"   freq error   ");
			process_but_releases();
			return;
		}

		if (eeprom.scan_step_freq == 0 || eeprom.scan_step_freq > (eeprom.end_freq - eeprom.start_freq))
		{
			OLED_ShowString(0, 6, (char *)"   step error   ");
			process_but_releases();
			return;
		}

		if (!ok_button.pressed_long)
		{
			if (eeprom.mode != FREQ_MODE_SPOT)
			{	// stpo sweeping
				eeprom.mode = FREQ_MODE_SPOT;
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				lcd_show_mode();
			}
		}
		else
		{
			sweep_hop_count = 0;
			sweep_hop_freq  = eeprom.start_freq;
			sweep_hop_next  = false;
			eeprom.mode     = FREQ_MODE_SWEEP;
			adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
		}

		eeprom_save();
		process_but_releases();

		task_id = TASK_NONE;

		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		if (p_index < (sizeof(buf) / 2))
			inc_buf(&buf[              0], sizeof(buf) / 2, p_index);
		else
			inc_buf(&buf[sizeof(buf) / 2], sizeof(buf) / 2, p_index - (sizeof(buf) / 2));
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		if (p_index < (sizeof(buf) / 2))
			dec_buf(&buf[              0], sizeof(buf) / 2, p_index);
		else
			dec_buf(&buf[sizeof(buf) / 2], sizeof(buf) / 2, p_index - (sizeof(buf) / 2));
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint64_t start_freq = buf_to_uint(&buf[              0], sizeof(buf) / 2);
			uint64_t end_freq   = buf_to_uint(&buf[sizeof(buf) / 2], sizeof(buf) / 2);

			if (start_freq < ADF4351_OUT_MIN_HZ)
				start_freq = ADF4351_OUT_MIN_HZ;

			if (end_freq > ADF4351_OUT_MAX_HZ)
				end_freq = ADF4351_OUT_MAX_HZ;

			eeprom.start_freq = start_freq;
			eeprom.end_freq   = end_freq;

			switch (eeprom.mode)
			{
				default:
				case FREQ_MODE_SPOT:
					adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
					break;
				case FREQ_MODE_SWEEP:
				case FREQ_MODE_HOP:
					adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					break;
			}
		}

		uint_to_buf(eeprom.start_freq, &buf[              0], sizeof(buf) / 2);
		uint_to_buf(eeprom.end_freq,   &buf[sizeof(buf) / 2], sizeof(buf) / 2);

		copy_freq_buf_2_display(&buf[              0], display1, p_index - 0, LEADING_ZERO_CHAR);
		OLED_ShowString(0, 2, (char *)"fr ");
		OLED_ShowString(CHAR_WIDTH * 3, 2, display1);

		copy_freq_buf_2_display(&buf[sizeof(buf) / 2], display2, p_index - (sizeof(buf) / 2), LEADING_ZERO_CHAR);
		OLED_ShowString(0, 4, (char *)"to ");
		OLED_ShowString(CHAR_WIDTH * 3, 4, display2);

		process_but_releases();
	}
}

// frequency hop start and end frequency settings
void hop_freq_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10 * 2];
	char display1[13];
	char display2[13];

	uint_to_buf(eeprom.start_freq, &buf[              0], sizeof(buf) / 2);
	uint_to_buf(eeprom.end_freq,   &buf[sizeof(buf) / 2], sizeof(buf) / 2);

	if (task_id != TASK_HOP_FREQ)
	{
		task_id = TASK_HOP_FREQ;

		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Hop Limits");

		copy_freq_buf_2_display(&buf[                0], display1, p_index - 0, LEADING_ZERO_CHAR);
		OLED_ShowString(0, 2, (char *)"fr ");
		OLED_ShowString(CHAR_WIDTH * 3, 2, display1);

		copy_freq_buf_2_display(&buf[(sizeof(buf) / 2)], display2, p_index - (sizeof(buf) / 2), LEADING_ZERO_CHAR);
		OLED_ShowString(0, 4, (char *)"to ");
		OLED_ShowString(CHAR_WIDTH * 3, 4, display2);

		lcd_show_mode();
	}

	if (ok_button.released)
	{
		if (eeprom.end_freq <= eeprom.start_freq)
		{
			OLED_ShowString(0, 6, (char *)"   freq error   ");
			process_but_releases();
			return;
		}

		if (eeprom.scan_step_freq == 0 || eeprom.scan_step_freq > (eeprom.end_freq - eeprom.start_freq))
		{
			OLED_ShowString(0, 6, (char *)"   step error   ");
			process_but_releases();
			return;
		}

		if (!ok_button.pressed_long)
		{
			if (eeprom.mode != FREQ_MODE_SPOT)
			{	// stop sweeping/hopping
				eeprom.mode = FREQ_MODE_SPOT;
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				lcd_show_mode();
			}
		}
		else
		{
			sweep_hop_count = 0;
			sweep_hop_freq  = eeprom.start_freq;
			sweep_hop_next  = false;
			eeprom.mode     = FREQ_MODE_HOP;
			adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
		}

		eeprom_save();
		process_but_releases();

		task_id = TASK_NONE;

		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		if (p_index < (sizeof(buf) / 2))
			inc_buf(&buf[              0], sizeof(buf) / 2, p_index);
		else
			inc_buf(&buf[sizeof(buf) / 2], sizeof(buf) / 2, p_index - (sizeof(buf) / 2));
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		if (p_index < (sizeof(buf) / 2))
			dec_buf(&buf[              0], sizeof(buf) / 2, p_index);
		else
			dec_buf(&buf[sizeof(buf) / 2], sizeof(buf) / 2, p_index - (sizeof(buf) / 2));
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint64_t start_freq = buf_to_uint(&buf[              0], sizeof(buf) / 2);
			uint64_t end_freq   = buf_to_uint(&buf[sizeof(buf) / 2], sizeof(buf) / 2);

			if (start_freq < ADF4351_OUT_MIN_HZ)
				start_freq = ADF4351_OUT_MIN_HZ;

			if (end_freq > ADF4351_OUT_MAX_HZ)
				end_freq = ADF4351_OUT_MAX_HZ;

			eeprom.start_freq = start_freq;
			eeprom.end_freq   = end_freq;

			switch (eeprom.mode)
			{
				default:
				case FREQ_MODE_SPOT:
					adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
					break;
				case FREQ_MODE_SWEEP:
				case FREQ_MODE_HOP:
					adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					break;
			}
		}

		uint_to_buf(eeprom.start_freq, &buf[              0], sizeof(buf) / 2);
		uint_to_buf(eeprom.end_freq,   &buf[sizeof(buf) / 2], sizeof(buf) / 2);

		copy_freq_buf_2_display(&buf[              0], display1, p_index - 0, LEADING_ZERO_CHAR);
		OLED_ShowString(0, 2, (char *)"fr ");
		OLED_ShowString(CHAR_WIDTH * 3, 2, display1);

		copy_freq_buf_2_display(&buf[sizeof(buf) / 2], display2, p_index - (sizeof(buf) / 2), LEADING_ZERO_CHAR);
		OLED_ShowString(0, 4, (char *)"to ");
		OLED_ShowString(CHAR_WIDTH * 3, 4, display2);

		process_but_releases();
	}
}

// Scan step size setting
void scan_step_freq_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10];
	char display[13];

	uint_to_buf(eeprom.scan_step_freq, buf, sizeof(buf));

	if (task_id != TASK_SCAN_STEP_FREQ)
	{
		task_id = TASK_SCAN_STEP_FREQ;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Frequency Step");
		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);
		lcd_show_mode();
		//LCD_Show_CEStr(0, 6, " ¡û¡ü BACK ¡ý¡ú ");
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		inc_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		dec_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint64_t freq = buf_to_uint(buf, sizeof(buf));

			if (freq < 250)
				freq = 250;
			else
			if (freq > ((eeprom.end_freq - eeprom.start_freq) / 2))
				freq =   (eeprom.end_freq - eeprom.start_freq) / 2;

			if (eeprom.mode != FREQ_MODE_SPOT)
			{	// stop sweeping
				eeprom.mode = FREQ_MODE_SPOT;
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				lcd_show_mode();
			}

			eeprom.scan_step_freq = freq;
		}

		uint_to_buf(eeprom.scan_step_freq, buf, sizeof(buf));

		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);

		process_but_releases();
	}
}

// Set scan speed
void scan_speed_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[4];
	char display[10];

	uint_to_buf(eeprom.scan_timer_ms, buf, sizeof(buf));

	if (task_id != TASK_SCAN_SPEED)
	{
		task_id = TASK_SCAN_SPEED;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Scan Speed");
		copy_time_buf_2_display(buf, display, p_index);
		OLED_ShowString(CHAR_WIDTH * 3, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		inc_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		dec_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint32_t sweep_timer_ms = buf_to_uint(buf, sizeof(buf));

			if (sweep_timer_ms < 1)
				sweep_timer_ms = 1;

			eeprom.scan_timer_ms = sweep_timer_ms;
		}

		uint_to_buf(eeprom.scan_timer_ms, buf, sizeof(buf));

		copy_time_buf_2_display(buf, display, p_index);
		OLED_ShowString(CHAR_WIDTH * 3, 3, display);

		process_but_releases();
	}
}

// Set output power
void output_power_set(void)
{
	uint8_t index = eeprom.dB_index;
	bool but_pressed = false;

	if (task_id != TASK_OUTPUT_POWER)
	{
		task_id = TASK_OUTPUT_POWER;

		OLED_Clear();
		LCD_Show_CEStr(0, 0, "RF Output Power");
		switch (index)
		{
			case 0:  LCD_Show_CEStr(CHAR_WIDTH * 3, 3, " off "); break;
			case 1:  LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "-4dBm"); break;
			case 2:  LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "-1dBm"); break;
			case 3:  LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "+2dBm"); break;
			case 4:  LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "+5dBm"); break;
		}
		lcd_show_mode();
		//LCD_Show_CEStr(0, 6, " ¡û¡ü BACK ¡ý¡ú ");
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		if (index < 4)
			index++;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		if (index > 0)
			index--;
	}

	if (but_pressed)
	{
		eeprom.dB_index = index;

		switch (eeprom.dB_index)
		{
			case 0: LCD_Show_CEStr(CHAR_WIDTH * 3, 3, " off "); break;
			case 1: LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "-4dBm"); break;
			case 2: LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "-1dBm"); break;
			case 3: LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "+2dBm"); break;
			case 4: LCD_Show_CEStr(CHAR_WIDTH * 3, 3, "+5dBm"); break;
		}

		adf4351_dev.params.output_power = eeprom.dB_index;

		switch (eeprom.mode)
		{
			default:
			case FREQ_MODE_SPOT:
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				break;
			case FREQ_MODE_SWEEP:
			case FREQ_MODE_HOP:
				adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
				break;
		}

		process_but_releases();
	}
}

// reference frequency setting
void ref_freq_set(void)
{
	uint8_t index = eeprom.ref_freq_table_index;
	bool but_pressed = false;
	uint8_t buf[10];
	char display[13];

	uint_to_buf(ref_freq_table[index].freq_in_Hz, buf, sizeof(buf));

	if (task_id != TASK_REF_FREQ)
	{
		task_id = TASK_REF_FREQ;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Ref Freq Input");
		copy_freq_buf_2_display(buf, display, 255, ' ');
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		if (index < ARRAY_SIZE(ref_freq_table) - 1)
			index++;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		if (index > 0)
			index--;
	}

	if (but_pressed)
	{
		const t_ref_freq ref_freq = ref_freq_table[index];

		eeprom.ref_freq_table_index = index;
		eeprom.ref_freq = ref_freq.freq_in_Hz;

		adf4351_dev.params.reference_freq_Hz     = eeprom.ref_freq;
		adf4351_dev.params.reference_div_factor  = ref_freq.r_divider;
		adf4351_dev.params.reference_mul2_enable = ref_freq.mul2_enable;
		adf4351_dev.params.reference_div2_enable = ref_freq.div2_enable;

		switch (eeprom.mode)
		{
			default:
			case FREQ_MODE_SPOT:
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				break;
			case FREQ_MODE_SWEEP:
			case FREQ_MODE_HOP:
				adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
				break;
		}

		uint_to_buf(eeprom.ref_freq, buf, sizeof(buf));

		copy_freq_buf_2_display(buf, display, 255, ' ');
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);

		process_but_releases();
	}
}

// reference input frequency fine adjust
void ref_freq_adjust_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10];
	char display[13];

	uint_to_buf(eeprom.ref_freq, buf, sizeof(buf));

	if (task_id != TASK_REF_FREQ_ADJUST)
	{
		task_id = TASK_REF_FREQ_ADJUST;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Ref Freq Adjust");
		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		inc_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		dec_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			const t_ref_freq ref_freq = ref_freq_table[eeprom.ref_freq_table_index];

			uint32_t freq = buf_to_uint(buf, sizeof(buf));

			// allow up to 20kHz adjustment either side .. should be more than enough to compensate for an xtal being off frequency ??
			if (freq < ref_freq.freq_in_Hz - 20000)
				freq = ref_freq.freq_in_Hz - 20000;
			else
			if (freq > ref_freq.freq_in_Hz + 20000)
				freq = ref_freq.freq_in_Hz + 20000;

			eeprom.ref_freq = freq;

			adf4351_dev.params.reference_freq_Hz = freq;

			switch (eeprom.mode)
			{
				default:
				case FREQ_MODE_SPOT:
					adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
					break;
				case FREQ_MODE_SWEEP:
				case FREQ_MODE_HOP:
					adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					break;
			}
		}

		uint_to_buf(eeprom.ref_freq, buf, sizeof(buf));

		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);

		process_but_releases();
	}
}

void channel_spacing_set(void)
{
	static uint8_t p_index = 0;

	bool but_pressed = false;
	bool changed = false;
	uint8_t buf[10];
	char display[13];

	uint_to_buf(eeprom.channel_spacing_freq, buf, sizeof(buf));

	if (task_id != TASK_CHAN_SPACE)
	{
		task_id = TASK_CHAN_SPACE;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Channel Spacing");
		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		inc_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		dec_buf(buf, sizeof(buf), p_index);
		changed = true;
	}
	else
	if (left_button.released)
	{
		but_pressed = true;
		if (p_index > 0)
			p_index--;
	}
	else
	if (right_button.released)
	{
		but_pressed = true;
		if (p_index < sizeof(buf) - 1)
			p_index++;
	}

	if (but_pressed)
	{
		if (changed)
		{
			uint64_t freq = buf_to_uint(buf, sizeof(buf));

			if (freq < 2500)
				freq = 2500;
			else
			if (freq > 1000000)
				freq = 1000000;

			eeprom.channel_spacing_freq = freq;

			adf4351_dev.params.channel_spacing_Hz = freq;

			switch (eeprom.mode)
			{
				default:
				case FREQ_MODE_SPOT:
					adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
					break;
				case FREQ_MODE_SWEEP:
				case FREQ_MODE_HOP:
					adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					break;
			}
		}

		uint_to_buf(eeprom.channel_spacing_freq, buf, sizeof(buf));

		copy_freq_buf_2_display(buf, display, p_index, LEADING_ZERO_CHAR);
		OLED_ShowString(CHAR_WIDTH * 2, 3, display);

		process_but_releases();
	}
}

void charge_pump_current_set(void)
{
	uint8_t index = eeprom.charge_pump_current_table_index;
	bool but_pressed = false;
	uint8_t buf[5];
	char display[8];

	uint_to_buf(charge_pump_current_table[index], buf, sizeof(buf));

	if (task_id != TASK_CHG_PUMP_CUR)
	{
		task_id = TASK_CHG_PUMP_CUR;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Charge Pump I");
		copy_current_buf_2_display(buf, display);
		OLED_ShowString(CHAR_WIDTH * 3, 3, display);
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		if (index < ARRAY_SIZE(charge_pump_current_table) - 1)
			index++;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		if (index > 0)
			index--;
	}

	if (but_pressed)
	{
		const uint16_t uA = charge_pump_current_table[index];

		eeprom.charge_pump_current_table_index = index;

		adf4351_dev.params.charge_pump_current = index;

		switch (eeprom.mode)
		{
			default:
			case FREQ_MODE_SPOT:
				adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
				break;
			case FREQ_MODE_SWEEP:
			case FREQ_MODE_HOP:
				adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
				break;
		}

		uint_to_buf(uA, buf, sizeof(buf));

		copy_current_buf_2_display(buf, display);
		OLED_ShowString(CHAR_WIDTH * 3, 3, display);

		process_but_releases();
	}
}

void unlock_mute_set(void)
{
	bool but_pressed = false;
	bool unlock_mute = eeprom.unlock_mute;

	if (task_id != TASK_UNLOCK_MUTE)
	{
		task_id = TASK_UNLOCK_MUTE;
		OLED_Clear();
		LCD_Show_CEStr(0, 0, "Unlock Mute");
		LCD_Show_CEStr(CHAR_WIDTH * 3, 3, (unlock_mute) ? " YES " : " no ");
		lcd_show_mode();
	}

	if (ok_button.released)
	{
		eeprom_save();
		process_but_releases();
		task_id = TASK_NONE;
		return;
	}

	if (up_button.released)
	{
		but_pressed = true;
		unlock_mute = true;
	}
	else
	if (down_button.released)
	{
		but_pressed = true;
		unlock_mute = false;
	}

	if (but_pressed)
	{
		if (eeprom.unlock_mute != unlock_mute)
		{
			eeprom.unlock_mute = unlock_mute;
			adf4351_dev.params.mute_till_lock_enable = unlock_mute;
			LCD_Show_CEStr(CHAR_WIDTH * 3, 3, (eeprom.unlock_mute) ? " YES " : " no ");
		}

		process_but_releases();
	}
}

// display menu selection
void lcd_show_menu(uint8_t start_info, uint8_t current_deal_info)
{
	const uint8_t max_lines = 3;
	uint8_t show_mode;
	uint16_t y_start = 0;

	//OLED_Clear();

	switch (start_info)
	{
		case TASK_FREQ:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Spot Frequency ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_SWEEP_FREQ:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Linear Sweep   ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_HOP_FREQ:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Random Hop     ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_SCAN_STEP_FREQ:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Frequency Step ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_SCAN_SPEED:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Scan Speed     ", show_mode);
			y_start += 2;
			if (y_start == max_lines * 2)
				break;

		case TASK_OUTPUT_POWER:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "RF Output Power", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_REF_FREQ:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Ref Freq Input ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_REF_FREQ_ADJUST:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Ref Freq Adjust", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_CHAN_SPACE:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Channel spacing", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_CHG_PUMP_CUR:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Charge Pump I  ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		case TASK_UNLOCK_MUTE:
			show_mode = ((y_start / 2) == (current_deal_info - start_info)) ? 0 : 1;
			LCD_Show_ModeCEStr(0, y_start, "Unlock Mute    ", show_mode);
			y_start += 2;
			if (y_start >= max_lines * 2)
				break;

		default:
			break;
	}

	lcd_show_mode();
}

void main_menu(void);

void menu_process(int new_menu)
{
	int menu_id = task_id;

	if (new_menu != TASK_NONE)
		menu_id = new_menu;	// switch to the new menu option

	switch (menu_id)
	{
		case TASK_FREQ:            freq_set();                break;
		case TASK_SWEEP_FREQ:      sweep_freq_set();          break;
		case TASK_HOP_FREQ:        hop_freq_set();            break;
		case TASK_SCAN_STEP_FREQ:  scan_step_freq_set();      break;
		case TASK_SCAN_SPEED:      scan_speed_set();          break;
		case TASK_OUTPUT_POWER:    output_power_set();        break;
		case TASK_REF_FREQ:        ref_freq_set();            break;
		case TASK_REF_FREQ_ADJUST: ref_freq_adjust_set();     break;
		case TASK_CHAN_SPACE:      channel_spacing_set();     break;
		case TASK_CHG_PUMP_CUR:    charge_pump_current_set(); break;
		case TASK_UNLOCK_MUTE:     unlock_mute_set();         break;
		default:                   task_id = TASK_NONE;
		case TASK_NONE:
		case TASK_MAIN_MENU:       main_menu();               break;
	}
}

// show the main menu
void main_menu(void)
{
	const uint8_t max_lines = 3;	// number of menu items to display
	bool draw_menu = false;

	if (task_id != TASK_MAIN_MENU)
	{
		task_id = TASK_MAIN_MENU;
		draw_menu = true;
	}
	else
	{
		if (ok_button.released)
		{
			if (ok_button.pressed_long)
			{
				switch (task_index)
				{
					case TASK_FREQ:
						if (eeprom.mode != FREQ_MODE_SPOT)
						{	// switch to spot frequency mode
							eeprom.mode = FREQ_MODE_SPOT;
							adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
							eeprom_save();
							lcd_show_mode();
						}
						break;

					case TASK_SWEEP_FREQ:
						if (eeprom.mode != FREQ_MODE_SWEEP)
						{	// switch to linear sweep mode
							sweep_hop_count = 0;
							sweep_hop_freq  = eeprom.start_freq;
							sweep_hop_next  = false;
							eeprom.mode     = FREQ_MODE_SWEEP;
							adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
							eeprom_save();
							lcd_show_mode();
						}
						break;

					case TASK_HOP_FREQ:
						if (eeprom.mode != FREQ_MODE_HOP)
						{	// switch to hop mode
							sweep_hop_count = 0;
							sweep_hop_next  = true;
							eeprom.mode     = FREQ_MODE_HOP;
							eeprom_save();
							lcd_show_mode();
						}
						break;

					default:	// do nothing
						break;
				}
			}
			else
			{	// switch to the selected sub-menu

				if (task_id != task_index)
				{
					process_but_releases();
					switch (task_index)
					{
						case TASK_FREQ:            freq_set();                break;
						case TASK_SWEEP_FREQ:      sweep_freq_set();          break;
						case TASK_HOP_FREQ:        hop_freq_set();            break;
						case TASK_SCAN_STEP_FREQ:  scan_step_freq_set();      break;
						case TASK_SCAN_SPEED:      scan_speed_set();          break;
						case TASK_OUTPUT_POWER:    output_power_set();        break;
						case TASK_REF_FREQ:        ref_freq_set();            break;
						case TASK_REF_FREQ_ADJUST: ref_freq_adjust_set();     break;
						case TASK_CHAN_SPACE:      channel_spacing_set();     break;
						case TASK_CHG_PUMP_CUR:    charge_pump_current_set(); break;
						case TASK_UNLOCK_MUTE:     unlock_mute_set();         break;
					}
					return;
				}
			}

			process_but_releases();
			return;
		}

		if (left_button.released)
		{	// step down in frequency
			if (eeprom.mode != FREQ_MODE_SPOT)
			{
				eeprom.mode = FREQ_MODE_SPOT;
			}
			else
			{
				eeprom.frequency -= eeprom.scan_step_freq;
				if (eeprom.frequency < ADF4351_OUT_MIN_HZ)
					eeprom.frequency = ADF4351_OUT_MIN_HZ;
				eeprom_save();
			}
			adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
			process_but_releases();
			show_menu_frequency();
			return;
		}

		if (right_button.released)
		{	// step up in frequency
			if (eeprom.mode != FREQ_MODE_SPOT)
			{
				eeprom.mode = FREQ_MODE_SPOT;
			}
			else
			{
				eeprom.frequency += eeprom.scan_step_freq;
				if (eeprom.frequency > ADF4351_OUT_MAX_HZ)
					eeprom.frequency = ADF4351_OUT_MAX_HZ;
				eeprom_save();
			}
			adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
			process_but_releases();
			show_menu_frequency();
			return;
		}

		if (up_button.released)
		{
			task_index = (task_index > 0) ? task_index - 1 : TASK_MAIN_MENU - 1;
			draw_menu = true;
		}
		else
		if (down_button.released)
		{
			task_index = (task_index < (TASK_MAIN_MENU - 1)) ? task_index + 1 : 0;
			draw_menu = true;
		}
	}

	if (draw_menu)
	{
		process_but_releases();
		uint8_t top_line = task_index - (max_lines - 1);
		if (top_line >= 128)
			top_line = 0;
		lcd_show_menu(top_line, task_index);
	}
}

// *********************************************************************

void process_rx_serial(t_rx_buffer *uart_rx)
{
	if (uart_rx == nullptr)
		return;

	if (uart_rx->buffer_wr > 0 && uart_rx->timer >= UART_RX_TIMEOUT_MS)
		uart_rx->buffer_wr = 0;	// rx timed out

	if (uart_rx->buffer_wr <= 0)
		return;						// nothing yet to process





	// TODO: handle all Rx'ed serial data that comes via the USB vitual comport and/or the hardware USART





}

// *********************************************************************
/*
void getCPU(void)
{
	#if defined(STM32F103xB)
		//const __IO uint32_t *cpu_id       = (uint32_t *)(0x411FC231    );	// readint this crashes the CPU
		const __IO uint8_t *bootloader_ID = (uint8_t  *)(0x1FFFF7D6    );	// see AN2606 page 25
	#elif defined(STM32F407xx)
		//const __IO uint32_t *cpu_id       = (uint32_t *)(0x411FC231    );
		const __IO uint8_t *bootloader_ID = (uint8_t  *)(0x1FFF77DE    );	// see AN2606 page 25
		const __IO uint32_t *package_type = (uint32_t *)(PACKAGE_BASE  );	//
	#endif
	const __IO uint16_t *flash_size_kB   = (uint16_t *)(FLASHSIZE_BASE);	// * 1024 for size in bytes
	//const __IO uint32_t *unique_ID0    = (uint32_t *)(UID_BASE + 0  );	// 96-bit
	//const __IO uint32_t *unique_ID1    = (uint32_t *)(UID_BASE + 4  );	//
	//const __IO uint32_t *unique_ID2    = (uint32_t *)(UID_BASE + 8  );	//

	memset(&cpu_details, 0, sizeof(cpu_details));
	//cpu_details.cpuID           = *cpu_id;
	cpu_details.sysclock        = HAL_RCC_GetSysClockFreq();
	cpu_details.hclock          = HAL_RCC_GetHCLKFreq();
	cpu_details.pclock1         = HAL_RCC_GetPCLK1Freq();
	cpu_details.pclock2         = HAL_RCC_GetPCLK2Freq();
	cpu_details.deviceID        = HAL_GetDEVID();
	cpu_details.revisionID      = HAL_GetREVID();
	cpu_details.flashSize_bytes = (uint32_t)(*flash_size_kB) * 1024;
	cpu_details.uniqueID_96[0]  = HAL_GetUIDw0();
	cpu_details.uniqueID_96[1]  = HAL_GetUIDw1();
	cpu_details.uniqueID_96[2]  = HAL_GetUIDw2();
	cpu_details.bootloader_id   = *bootloader_ID;
	#if defined(STM32F407xx)
		cpu_details.package_type = *package_type;	// 0x289fd76
	#endif
}
*/
// *********************************************************************

int main(void)
{
/*	// fetch the reset cause
	const bool reset_por    = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST ) != RESET) ? true : false;
	const bool reset_pin    = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST ) != RESET) ? true : false;
	const bool reset_sft    = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST ) != RESET) ? true : false;
	const bool reset_iwdg   = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? true : false;
	const bool reset_wwdg   = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) ? true : false;
	const bool reset_lpwr   = (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) ? true : false;
	const bool reset_lsirdy = (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY ) != RESET) ? true : false;
*/	__HAL_RCC_CLEAR_RESET_FLAGS();			// reset flags ready for next reboot

	HAL_Init();

	SystemClock_Config();

	#ifdef USE_IWDG
		MX_IWDG_Init();
	#endif
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	#ifdef USE_USB_VCP
		MX_USB_DEVICE_Init();
	#endif

	// disable stdout printf() buffering
	setbuf(stdout, nullptr);
	//setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

	//getCPU();

	makeCRC16Table();

	// default settings
	memset(&eeprom, 0xff, sizeof(eeprom));
	eeprom.scan_timer_ms                   = 1;               //
	eeprom.scan_step_freq                  = 12500;           //
	eeprom.start_freq                      = 144000000;       //
	eeprom.end_freq                        = 146000000;       //
	eeprom.frequency                       = 145000000;       //
	eeprom.channel_spacing_freq            = 3125;            //
	eeprom.dB_index                        = 3;               // 0 == off, 1 == -4dB, 2 = -1dB, 3 = +2dB, 4 = +5dB
	eeprom.mode                            = FREQ_MODE_SPOT;  //
	eeprom.ref_freq_table_index            = 19;              // 100 MHz
	eeprom.ref_freq                        = ref_freq_table[eeprom.ref_freq_table_index].freq_in_Hz;
	eeprom.charge_pump_current_table_index = 7;               // 2.5mA

	#ifdef USE_USB_VCP
		memset(&usb_rx, 0, sizeof(usb_rx));
		usb_rx.timer = 0xffff;
	#endif

	memset(&uart1_rx, 0, sizeof(uart1_rx));
	uart1_rx.timer = 0xffff;

/*
	printf("title: %s v%x.%02x\r\n", title, (version >> 8) & 0xff, (version >> 0) & 0xff);

	printf("reset: %s %s %s %s %s %s %s\n",
		reset_por    ? "POR"    : "por",
		reset_pin    ? "PIN"    : "pin",
		reset_sft    ? "SFT"    : "sft",
		reset_iwdg   ? "IWDG"   : "iwdg",
		reset_wwdg   ? "WWdg"   : "wwdg",
		reset_lpwr   ? "LPWR"   : "lpwr",
		reset_lsirdy ? "LSIRDY" : "lsirdy");
*/

	// set the initial but states
	button_debouce_set(&left_button,  (HAL_GPIO_ReadPin(BUT_LEFT_GPIO_Port,  BUT_LEFT_Pin)  == GPIO_PIN_RESET) ? true : false);
	button_debouce_set(&right_button, (HAL_GPIO_ReadPin(BUT_RIGHT_GPIO_Port, BUT_RIGHT_Pin) == GPIO_PIN_RESET) ? true : false);
	button_debouce_set(&up_button,    (HAL_GPIO_ReadPin(BUT_UP_GPIO_Port,    BUT_UP_Pin)    == GPIO_PIN_RESET) ? true : false);
	button_debouce_set(&down_button,  (HAL_GPIO_ReadPin(BUT_DOWN_GPIO_Port,  BUT_DOWN_Pin)  == GPIO_PIN_RESET) ? true : false);
	button_debouce_set(&ok_button,    (HAL_GPIO_ReadPin(BUT_OK_GPIO_Port,    BUT_OK_Pin)    == GPIO_PIN_RESET) ? true : false);

	OLED_Init();
	OLED_Clear();

	// check for a button being held down at boot up
	if (ok_button.down || left_button.down || right_button.down || up_button.down || down_button.down)
	{	// a button is held down
		uint32_t tick;

		if (ok_button.down)	    // butt OK ?
		{
//			printf("Erasing EEPROM ..");

			LCD_Show_ModeCEStr(0, 2, "ERASING EEPROM ", 1);

			#ifdef USE_IWDG
				service_IWDG(true);
			#endif

			STMFLASH_Erase();	    // wipe the emulated eeprom flash sector

			OLED_Clear();
			LCD_Show_ModeCEStr(0, 2, " EEPROM erased ", 1);

//			printf(" done\n");
		}

		LCD_Show_ModeCEStr(0, 5, "Release button ", 1);

		// wait for all buttons to be released
		tick = HAL_GetTick();
		while ((HAL_GetTick() - tick) < 200)	// 200ms debounce
		{
			__WFI();		// wait here until an interrupt nudges us

			if (ok_button.down || left_button.down || right_button.down || up_button.down || down_button.down)
				tick = HAL_GetTick();

			#ifdef USE_IWDG
				service_IWDG(false);
			#endif
		}

		OLED_Clear();

		button_debouce_reset(&ok_button);
		button_debouce_reset(&left_button);
		button_debouce_reset(&right_button);
		button_debouce_reset(&up_button);
		button_debouce_reset(&down_button);
	}

	// fetch our saved settings (if any)
	eeprom_load();

	{	// setup the ADF4351
		const t_ref_freq ref_freq = ref_freq_table[eeprom.ref_freq_table_index];

		adf4351_init_defaults(&adf4351_dev);
		adf4351_dev.params.reference_freq_Hz     = eeprom.ref_freq;
		adf4351_dev.params.channel_spacing_Hz    = eeprom.channel_spacing_freq;
		adf4351_dev.params.frequency_Hz          = eeprom.frequency;
		adf4351_dev.params.reference_div_factor  = ref_freq.r_divider;
		adf4351_dev.params.reference_mul2_enable = ref_freq.mul2_enable;
		adf4351_dev.params.reference_div2_enable = ref_freq.div2_enable;
		//adf4351_dev.params.gcd_enable          = false;
		adf4351_dev.params.mute_till_lock_enable = true;
		adf4351_dev.params.output_power          = eeprom.dB_index;
		adf4351_dev.params.charge_pump_current   = eeprom.charge_pump_current_table_index;

		adf4351_init(&adf4351_dev);
	}

	if (	(eeprom.mode == FREQ_MODE_SWEEP || eeprom.mode == FREQ_MODE_HOP) &&
			eeprom.start_freq < eeprom.end_freq &&
			eeprom.start_freq >= ADF4351_OUT_MIN_HZ &&
			eeprom.end_freq   <= ADF4351_OUT_MAX_HZ &&
			eeprom.scan_step_freq > 0 &&
			eeprom.scan_step_freq <= (eeprom.end_freq - eeprom.start_freq))
	{	// start sweeping
		sweep_hop_freq = eeprom.start_freq;
		adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
	}
	else
	{	// not sweeping
		if (eeprom.mode != FREQ_MODE_SPOT)
		{
			eeprom.mode = FREQ_MODE_SPOT;
			eeprom_save();
			adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
		}
	}

	{	// show a boot message on the display
		char str[17];
		sprintf(str, "ADF4351 v%u.%03u ", version >> 8, version & 0xff);
		LCD_Show_ModeCEStr(0, 2, (const char *)str, 1);
		//LCD_Show_ModeCEStr(0, 4, "    by G6AMU   ", 1);

		#ifdef USE_IWDG
			service_IWDG(true);
		#endif

		delay_ms(1500);
	}

	// show the main menu
	menu_process(TASK_MAIN_MENU);

	while (1)
	{
		__DSB();
		__WFI();		// Wait For Interrupt .. we get to go to sleep here :)

		menu_process(TASK_NONE);

		#ifdef USE_USB_VCP
			process_rx_serial(&usb_rx);
		#endif
		process_rx_serial(&uart1_rx);

		bool comms_time_out = true;
		if (uart1_rx.timer < CONNECTION_TIMEOUT_MS)
			comms_time_out = false;
		#ifdef USE_USB_VCP
			if (usb_rx.timer < CONNECTION_TIMEOUT_MS)
				comms_time_out = false;
		#endif

		if (!comms_time_out)
		{

			// TODO: send stuff to the PC side client software using the USB serial connection and/or the hardware USART

		}

		switch (eeprom.mode)
		{
			default:
				eeprom.mode = FREQ_MODE_SPOT;
			case FREQ_MODE_SPOT:
				break;

			case FREQ_MODE_SWEEP:		// step to the next sweep frequency
				if (sweep_hop_next)
				{
					sweep_hop_next = false;
					if (	eeprom.start_freq < eeprom.end_freq &&
							eeprom.start_freq > 0 &&
							eeprom.scan_step_freq > 0 &&
							eeprom.scan_step_freq <= (eeprom.end_freq - eeprom.start_freq))
					{	// compute the next frequency to jump too
						uint64_t freq = sweep_hop_freq + eeprom.scan_step_freq;
						if (freq < eeprom.start_freq || freq > eeprom.end_freq)
							freq = eeprom.start_freq;
						sweep_hop_freq = freq;
						adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					}
					else
					{	// error .. stop the sweep
						eeprom.mode = FREQ_MODE_SPOT;
						adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
						lcd_show_mode();
					}
				}
				break;

			case FREQ_MODE_HOP:			// step to the next hop frequency
				if (sweep_hop_next)
				{
					sweep_hop_next = false;
					if (	eeprom.start_freq < eeprom.end_freq &&
							eeprom.start_freq > 0 &&
							eeprom.scan_step_freq > 0 &&
							eeprom.scan_step_freq <= (eeprom.end_freq - eeprom.start_freq))
					{	// pick a random channel to jump too
						const uint32_t num_channels = 1 + ((eeprom.end_freq - eeprom.start_freq) / eeprom.scan_step_freq);
						while (1)
						{
							// generate a pseudo random 32-bit number using the internal CRC unit
							hcrc.Instance->DR = 0x12345678;
							random32 = hcrc.Instance->DR;
							#ifdef DWT
								//hcrc.Instance->DR = DWT->CYCCNT;
								//random32 ^= hcrc.Instance->DR;
							#endif

							//const uint32_t channel = rand() % num_channels;
							const uint32_t channel = random32 % num_channels;
							const uint64_t freq = eeprom.start_freq + (eeprom.scan_step_freq * channel);
							if (sweep_hop_freq != freq)
							{
								sweep_hop_freq = freq;
								break;
							}
						}
						adf4351_set_freq(&adf4351_dev, sweep_hop_freq, false);
					}
					else
					{	// error .. stop the hop
						eeprom.mode = FREQ_MODE_SPOT;
						adf4351_set_freq(&adf4351_dev, eeprom.frequency, false);
						lcd_show_mode();
					}
				}
				break;
		}

		#ifdef USE_IWDG
			service_IWDG(false);
		#endif
	}

	reboot();
}
