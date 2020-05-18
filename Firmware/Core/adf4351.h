
#ifndef __ADF4351_H
#define __ADF4351_H

//#pragma once

#include "common.h"

#ifdef __cplusplus
	extern "C" {
#endif

#define ADF4351_OUT_MAX_HZ				4400000000ull
#define ADF4351_OUT_MIN_HZ				34375000

#define ADF4351_REFIN_MAX_HZ			250000000
#define ADF4351_REFIN_MIN_HZ			10000000

#define ADF4351_OUTPUT_POWER_MAX		4					// 0 = output disabled

typedef struct
{
	uint32_t magic;    // set to a specific value simply so that we know wwe have beenn initialised

	struct
	{
		uint32_t reference_freq_Hz;                        // the input reference frequency
		uint32_t channel_spacing_Hz;                       // used in the fractional divider calculation
		uint64_t frequency_Hz;                             // the desired output frequency
		uint16_t reference_div_factor;                     // 10-bit reference input divider value .. 1 to 1023
		bool     reference_mul2_enable;                    // false = ref * 1, true = ref * 2
		bool     reference_div2_enable;                    // false = ref * 1, true = ref / 2
		bool     gcd_enable;                               // false = don't bother with GCD, true = run the FRAC and MOD values through the GCD
		bool     phase_detector_polarity_positive_enable;  // false = negative PD polarity, true = positive PD polarity
		bool     lock_detect_precision_6ns_enable;         // false = 10ns, true = 6ns
		bool     lock_detect_function_integer_n_enable;    // false = fractional pll, true = integer pll
		uint8_t  charge_pump_current;                      // 7 = 2.5mA (0 to 15 = 0.31mA to 5mA)
		uint8_t  muxout_select;                            // 0 = 3-state out, 1 = Dvdd, 2 = DGND, 3 = R-counter out, 4 = N-div out, 5 = analogue lock detect out, 6 = digital lock detect out
		bool     low_spur_mode_enable;                     // false = lower noise mode, true = lower spur mode
		bool     cycle_slip_reduction_enable;              // true = slip ur cycles !
		bool     charge_cancellation_enable;               // true = cancel ur charge !
		bool     anti_backlash_3ns_enable;                 // false = 6ns (FRAC-N), true = 3ns (INT-N)
		bool     band_select_clock_mode_high_enable;       // false = low, true = high
		uint16_t clock_divider_value;                      // 12-bit clock divider value
		uint8_t  clock_divider_mode;                       // 0 = clk div off, 1 = fast lock enable, 2 = resync enable
		uint8_t  output_power;                             // 0 = disable output, 1 = -4dBm, 2 = -1dBm, 3 = +2dBm, 4 = +5dBm
		uint8_t  aux_output_power;                         // 0 = disable output, 1 = -4dBm, 2 = -1dBm, 3 = +2dBm, 4 = +5dBm
		bool     aux_output_select;                        // false = divided output, true = fundamental
		bool     mute_till_lock_enable;                    // false = output enabled when VCO unlocked, true = output disabled when VCO unlocked

		//SPI_HandleTypeDef *spi_port;

		//GPIO_TypeDef *ld_port;
		//uint32_t      ld_pin;

		//GPIO_TypeDef *ce_port;
		//uint32_t      ce_pin;

		//GPIO_TypeDef *le_port;
		//uint32_t      le_pin;

		//GPIO_TypeDef *data_port;
		//uint32_t      data_pin;

		//GPIO_TypeDef *clk_port;
		//uint32_t      clk_pin;
	} params;

	// the ADF4351 internal values that we compute
	uint64_t frequency_Hz;       // the current frequency
	uint64_t vco_Hz;
	uint32_t channel_spacing_Hz;
	uint16_t r_counter;
	uint32_t pfd_freq_Hz;
	uint16_t frac_div;
	uint16_t int_div;
	uint16_t mod;
	uint8_t  band_sel_div;
	uint32_t band_select_Hz;
	uint8_t  output_divider;
	uint16_t phase_value;
	uint16_t min_mod;
	uint32_t min_int_div;
	//float min_step_Hz;

	// the ADF4351 internal register values
	uint32_t regs[6];
	uint32_t regs_sent[6];

} t_adf4351_dev;

int adf4351_write_reg(uint32_t data);

uint64_t adf4351_get_freq(t_adf4351_dev *dev);
uint64_t adf4351_set_freq(t_adf4351_dev *dev, uint64_t freq_Hz, bool force);

#if defined(ADF4351_LD_GPIO_Port) && defined(ADF4351_LD_Pin)
	bool adf4351_locked(t_adf4351_dev *dev, uint32_t timeout_us);
#endif

int adf4351_init(t_adf4351_dev *dev);

void adf4351_init_defaults(t_adf4351_dev *dev);

#ifdef __cplusplus
	}
#endif

#endif
