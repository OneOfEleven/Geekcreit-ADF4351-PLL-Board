
#ifndef __COMMON_H
#define __COMMON_H

//#pragma once

#include <stdint.h>
#include <stdbool.h>

//#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//#pragma GCC diagnostic pop

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
	extern "C" {
#endif

#ifndef nullptr
	#define nullptr               NULL
#endif

#define OPTIMIZE_NONE            __attribute__((optimize("-O0")))
#ifdef _DEBUG
	#define OPTIMIZE_MOST
	#define OPTIMIZE_SIZE
	#define OPTIMIZE_SPEED
#else
	#define OPTIMIZE_MOST         __attribute__((optimize("-O3")))
	#define OPTIMIZE_SIZE         __attribute__((optimize("-Os")))
	#define OPTIMIZE_SPEED        __attribute__((optimize("-Ofast")))
#endif

#define ARRAY_SIZE(array)        (sizeof(array) / sizeof(array[0]))

#ifndef _DEBUG
	#define USE_IWDG
#endif

#define USE_USB_VCP

#define BUTTON_DEBOUNCE_MS       30		// 1 to 255
#define BUTTON_DEBOUNCE_LONG_MS  500	// 2 to 65535

#define EEPROM_MAGIC             0xADF4351A

#define SERIAL_BAUDRATE          115200
//#define SERIAL_BAUDRATE        230400
//#define SERIAL_BAUDRATE        250000
//#define SERIAL_BAUDRATE        460800
//#define SERIAL_BAUDRATE        500000
//#define SERIAL_BAUDRATE        921600
//#define SERIAL_BAUDRATE        1000000
//#define SERIAL_BAUDRATE        1843200
//#define SERIAL_BAUDRATE        2000000
//#define SERIAL_BAUDRATE        3000000
//#define SERIAL_BAUDRATE        3686400
//#define SERIAL_BAUDRATE        4000000

// *******************************************************
// pins

#define XTAL_IN_Pin              GPIO_PIN_0
#define XTAL_IN_GPIO_Port        GPIOD

#define XTAL_OUT_Pin             GPIO_PIN_1
#define XTAL_OUT_GPIO_Port       GPIOD

#define BUT_LEFT_Pin             GPIO_PIN_1
#define BUT_LEFT_GPIO_Port       GPIOA

#define BUT_OK_Pin               GPIO_PIN_2
#define BUT_OK_GPIO_Port         GPIOA

#define BUT_DOWN_Pin             GPIO_PIN_3
#define BUT_DOWN_GPIO_Port       GPIOA

#define BUT_RIGHT_Pin            GPIO_PIN_0
#define BUT_RIGHT_GPIO_Port      GPIOB

#define BUT_UP_Pin               GPIO_PIN_1
#define BUT_UP_GPIO_Port         GPIOB

#define OLED_DC_Pin              GPIO_PIN_4
#define OLED_DC_GPIO_Port        GPIOA

#define OLED_RST_Pin             GPIO_PIN_5
#define OLED_RST_GPIO_Port       GPIOA

#define OLED_CLK_Pin             GPIO_PIN_6
#define OLED_CLK_GPIO_Port       GPIOA

#define OLED_SDA_Pin             GPIO_PIN_7
#define OLED_SDA_GPIO_Port       GPIOA

#define ADF4351_CE_Pin           GPIO_PIN_12
#define ADF4351_CE_GPIO_Port     GPIOB

#define ADF4351_LE_Pin           GPIO_PIN_13
#define ADF4351_LE_GPIO_Port     GPIOB

#define ADF4351_DATA_Pin         GPIO_PIN_14
#define ADF4351_DATA_GPIO_Port   GPIOB

#define ADF4351_CLK_Pin          GPIO_PIN_15
#define ADF4351_CLK_GPIO_Port    GPIOB

#define ADF4351_LD_Pin           GPIO_PIN_8
#define ADF4351_LD_GPIO_Port     GPIOA

#define UART_TX_Pin              GPIO_PIN_9
#define UART_TX_GPIO_Port        GPIOA

#define UART_RX_Pin              GPIO_PIN_10
#define UART_RX_GPIO_Port        GPIOA

#define USB_DM_Pin               GPIO_PIN_11
#define USB_DM_GPIO_Port         GPIOA

#define USB_DP_Pin               GPIO_PIN_12
#define USB_DP_GPIO_Port         GPIOA

#define SWDIO_Pin                GPIO_PIN_13
#define SWDIO_GPIO_Port          GPIOA

#define SWCLK_Pin						GPIO_PIN_14
#define SWCLK_GPIO_Port				GPIOA

#define LED1_Pin                 GPIO_PIN_5
#define LED1_GPIO_Port           GPIOB

// *******************************************************

#pragma pack(push, 1)
	typedef struct t_eeprom
	{
		uint32_t magic;                           // used to mark the start of a valid block in emulated eeprom (CPU flash)
		uint64_t scan_step_freq;                  // Hz
		uint64_t start_freq;                      // Hz
		uint64_t end_freq;                        // Hz
		uint64_t frequency;                       // Hz
		uint32_t ref_freq;                        // Hz
		uint32_t channel_spacing_freq;            // Hz
		uint16_t scan_timer_ms;                   // >= 1
		uint8_t  dB_index;                        // 2 to 5
		uint8_t  mode;                            // '0' = spot frequency, '1' = sweep, '2' = hop
		uint8_t  ref_freq_table_index;            // >= 0
		uint8_t  charge_pump_current_table_index; // 0 to 15
		bool     unlock_mute;							// false / true
		bool     gcd_enable;								// false / true
		uint8_t  padding[2];                      // make it a multiple of 32-bit words in size
		uint16_t crc;                             // must be the last variable in the block
	} t_eeprom;
#pragma pack(pop)

typedef struct t_button
{
	volatile uint8_t  debounce_tick;
	volatile uint16_t long_press_tick;
	volatile bool     down;
	volatile bool     down_long;
	volatile bool     pressed;
	volatile bool     pressed_long;
	volatile bool     released;
} t_button;

// *******************************************************

extern UART_HandleTypeDef    huart1;
#ifdef USE_USB_VCP
	extern PCD_HandleTypeDef  hpcd_USB_FS;
#endif
#ifdef USE_IWDG
	extern IWDG_HandleTypeDef hiwdg;
#endif
extern CRC_HandleTypeDef     hcrc;

extern t_eeprom eeprom;

extern t_button left_button;
extern t_button right_button;
extern t_button up_button;
extern t_button down_button;
extern t_button ok_button;

//t_cpu getCPU(void);
void delay_ms(uint32_t ms);
void delay_ns(uint32_t ns);

#ifdef __cplusplus
	}
#endif

// *******************************************************

#endif
