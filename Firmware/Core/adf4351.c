
#include "adf4351.h"

//#define ADF4351_SPI_PORT       hspi1			// the SPI port for the ADF4351

#define ADF4351_MAGIC                  0x0ADF4351	// any 32-bit magic value you like

#define ADF4351_VCO_MIN_HZ             2200000000ull

#define ADF4351_45_PRESCALER_MAX_HZ    3600000000ull
#define ADF4351_PFD_FRAC_MAX_HZ        32000000
#define ADF4351_PFD_INT_MAX_HZ         90000000

#define ADF4351_BANDSEL_HIGH_MAX_HZ    500000
#define ADF4351_BANDSEL_LOW_MAX_HZ		125000

#define ADF4351_MODULUS_MAX            4095

#define ADF4351_FRAC_MAX               4095

#define ADF4351_R_COUNTER_MAX          1023
#define ADF4351_R_COUNTER_MIN          1

#define ADF4351_OUT_DIV_MAX            6

#if defined(ADF4351_SPI_PORT)
	// write 32-bits of data to the ADF4351
	int adf4351_write_reg(const uint32_t data)
	{
		uint8_t tx_data[4];
		tx_data[0] = (data >> 24) & 0xff;
		tx_data[1] = (data >> 16) & 0xff;
		tx_data[2] = (data >>  8) & 0xff;
		tx_data[3] = (data >>  0) & 0xff;

		if (ADF4351_SPI_PORT.Instance == nullptr)
			return -1;

		// ADF4351 is 20MHz max SPI clock speed

		delay_ns(50);
		HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_RESET);

			delay_ns(50);
			const HAL_StatusTypeDef res = HAL_SPI_Transmit(&ADF4351_SPI_PORT, &tx_data[0], sizeof(tx_data), 10);

		delay_ns(50);
		HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_SET);

		return (res == HAL_OK) ? 0 : -2;	// 0 = OK
	}
#else
	// write 32-bits of data to the ADF4351
	int adf4351_write_reg(const uint32_t data)
	{
		uint8_t tx_data[4];

		tx_data[0] = (data >> 24) & 0xff;
		tx_data[1] = (data >> 16) & 0xff;
		tx_data[2] = (data >>  8) & 0xff;
		tx_data[3] = (data >>  0) & 0xff;

		HAL_GPIO_WritePin(ADF4351_CLK_GPIO_Port, ADF4351_CLK_Pin, GPIO_PIN_RESET);

		delay_ns(50);
		HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_RESET);

		{
			int reg;
			for (reg = 0; reg < (int)sizeof(tx_data); reg++)
			{
				int i;
				uint8_t b = tx_data[reg];
				for (i = 0; i < 8; i++)
				{
					delay_ns(50);
					HAL_GPIO_WritePin(ADF4351_DATA_GPIO_Port, ADF4351_DATA_Pin, ((b & 0x80) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

					delay_ns(50);
					HAL_GPIO_WritePin(ADF4351_CLK_GPIO_Port, ADF4351_CLK_Pin, GPIO_PIN_SET);

					delay_ns(50);
					HAL_GPIO_WritePin(ADF4351_CLK_GPIO_Port, ADF4351_CLK_Pin, GPIO_PIN_RESET);

					b <<= 1;
				}
			}
		}

		delay_ns(50);
		HAL_GPIO_WritePin(ADF4351_DATA_GPIO_Port,    ADF4351_DATA_Pin, GPIO_PIN_RESET);

		delay_ns(50);
		HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_SET);

		return 0;
	}
#endif

/*
uint32_t gcd(uint32_t u, uint32_t v)
{	// calculate the (G)reatest (C)ommon (D)ivisor of two numbers .. this is recursive so be careful
	if (u == v)
		return u;
	if (u == 0)
		return v;
	if (v == 0)
		return u;
	if (~u & 1)
		return (v & 1) ? gcd(u >> 1, v) : gcd(u >> 1, v >> 1) << 1;
	if (~v & 1)
		return gcd(u, v >> 1);
	return (u > v) ? gcd((u - v) >> 1, v) : gcd((v - u) >> 1, u);
}
*/

int adf4351_set_regs(t_adf4351_dev *dev, bool enable_output, bool force)
{
	uint32_t reg;

	if (dev == nullptr)
		return -1;
	if (dev->magic != ADF4351_MAGIC)
		return -2;

	// register 0
	reg = 0;                                                               //
	reg |= (uint32_t)(dev->frac_div & 0xfff) << 3;                         // 12-bit fractional part
	reg |= (uint32_t)(dev->int_div & 0xffff) << 15;                        // 16-bit integer part
	dev->regs[0] = reg;                                                    //

	// register 1
	reg = 0;                                                               //
	reg |= (uint32_t)(dev->mod & 0xfff) << 3;                              // 12-bit MOD value
	reg |= 1u << 15;                                                       // 12-bit phase value
	if (dev->vco_Hz > ADF4351_45_PRESCALER_MAX_HZ)                         //
		reg |= 1u << 27;                                                    // prescaler = 8/9
	//reg |= 1u << 28;                                                     // phase adjust on
	dev->regs[1] = reg;                                                    //

	// register 2
	reg = 0;                                                               //
	//reg |= 1u << 3;                                                      // counter reset disable
	//reg |= 1u << 4;                                                      // charge pump 3 state disable
	if (!enable_output && dev->frequency_Hz == 0)                          //
		reg |= 1u << 5;                                                     // power down
	if (dev->params.phase_detector_polarity_positive_enable)               //
		reg |= 1u << 6;                                                     // PD polarity positive
	if (dev->params.lock_detect_precision_6ns_enable)                      //
		reg |= 1u << 7;                                                     // LDP 6ns
	//reg |= 1u << 8;                                                      // LDF INT-N
	reg |= (uint32_t)(dev->params.charge_pump_current & 0xf) << 9;         // 4-bit charge pump current normally 2.5mA
	reg |= 1u << 13;                                                       // enable register double buffer
	reg |= (uint32_t)(dev->r_counter & 0x3ff) << 14;                       // 10-bit R counter
	if (dev->params.reference_div2_enable)                                 //
		reg |= 1u << 24;                                                    // reference div-by-2
	if (dev->params.reference_mul2_enable)                                 //
		reg |= 1u << 25;                                                    // reference mul-by-2
	reg |= (uint32_t)(dev->params.muxout_select & 0x7) << 26;              // mux-out = digital lock detect
	if (dev->params.low_spur_mode_enable)                                  //
		reg |= 3u << 29;                                                    // low spur mode
	dev->regs[2] = reg;                                                    //

	// register 3
	reg = 0;                                                               //
	reg |= (uint32_t)(dev->params.clock_divider_value & 0xfff) << 3;       // 12-bit clock divider
	reg |= (uint32_t)(dev->params.clock_divider_mode & 0x3) << 15;         // 2-bit clock divider mode
	if (dev->params.cycle_slip_reduction_enable)                           //
		reg |= 1u << 18;                                                    // cycle slip reduction enable
	if (dev->params.charge_cancellation_enable)                            //
		reg |= 1u << 21;                                                    // charge cancellation enable
	if (dev->params.anti_backlash_3ns_enable)                              //
		reg |= 1u << 22;                                                    // anti-backlash pulse with 3ns (INT-N)
	if (dev->params.band_select_clock_mode_high_enable)                    //
		reg |= 1u << 23;                                                    // band select clock mode high
	dev->regs[3] = reg;                                                    //

	// register 4
	reg = 0;                                                               //
	if (	enable_output &&                                                 //
			dev->params.output_power > 0 &&                                  //
			dev->frequency_Hz > 0)                                           //
	{                                                                      //
		reg |= (uint32_t)((dev->params.output_power - 1) & 0x3) << 3;       // 2-bit output power
		reg |= 1u << 5;                                                     // enable output
	}                                                                      //
	if (	enable_output &&                                                 //
			dev->params.aux_output_power > 0 &&                              //
			dev->frequency_Hz > 0)                                           //
	{                                                                      //
		reg |= (uint32_t)((dev->params.aux_output_power - 1) & 0x3) << 6;   // 2-bit AUX output power
		reg |= 1u << 8;                                                     // enable AUX output
		if (dev->params.aux_output_select)                                  //
			reg |= 1u << 9;                                                  // AUX output select = fundamental
	}                                                                      //
	if (dev->params.mute_till_lock_enable)                                 //
		reg |= 1u << 10;                                                    // mute till lock detect
	if (	!enable_output &&                                                //
			dev->params.output_power == 0 &&                                 //
			dev->params.aux_output_power == 0 &&                             //
			dev->frequency_Hz == 0)                                          //
		reg |= 1u << 11;                                                    // VCO power down
	reg |= (uint32_t)(dev->band_sel_div & 0xff) << 12;                     // 8-bit band select clock divider
	reg |= (uint32_t)(dev->output_divider & 0x7) << 20;                    // 3-bit output divider value
	reg |= 1u << 23;                                                       // select the VCO fundamental for the PLL feedback
	dev->regs[4] = reg;                                                    //

	// register 5
	reg = 0;                                                               //
	reg |= 3u << 19;                                                       // reserved
	reg |= 1u << 22;                                                       // lock detect pin mode = digital lock detect
	dev->regs[5] = reg;                                                    //

	{	// write the six register values to the chip
		bool changed = force;
		if (!changed)
		{
			int i = 0;
			while (i++ < 6 && !changed)
				if (dev->regs_sent[i] != dev->regs[i])
					changed = true;
		}
		if (changed)
		{
			int i = 6;
			while (--i >= 0)
			{
				const uint32_t value = dev->regs[i];
				if (force || i == 4 || i <= 1 || dev->regs_sent[i] != value)
				{
					if (adf4351_write_reg(value | (uint32_t)i) < 0)
						return -3;                   // error
					dev->regs_sent[i] = value;      // remember what we've sent
				}
			}
		}
	}

	return 0;	// OK
}

uint64_t adf4351_get_freq(t_adf4351_dev *dev)
{
	uint64_t freq;
	if (dev == nullptr)
		return 0;
	if (dev->magic != ADF4351_MAGIC)
		return 0;
	freq = (dev->magic == ADF4351_MAGIC && dev->mod > 0) ? ((uint64_t)((dev->int_div * dev->mod) + dev->frac_div) * dev->pfd_freq_Hz) / (dev->mod * (1u << dev->output_divider)) : 0;
	return freq;
}

// return the actual frequency value that was set
uint64_t adf4351_set_freq(t_adf4351_dev *dev, uint64_t freq_Hz, bool force)
{
	if (dev == nullptr)
		return 0;
	if (dev->magic != ADF4351_MAGIC)
		return 0;

	dev->frequency_Hz       = 0;
	dev->vco_Hz             = 0;
	dev->channel_spacing_Hz = dev->params.channel_spacing_Hz;
	dev->r_counter          = dev->params.reference_div_factor;
	dev->pfd_freq_Hz        = 0;
	dev->frac_div           = 0;
	dev->int_div            = 0;
	dev->mod                = 0;
	dev->band_sel_div       = 0;
	dev->output_divider     = 0;

	if (freq_Hz == 0)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	if (freq_Hz < ADF4351_OUT_MIN_HZ || freq_Hz > ADF4351_OUT_MAX_HZ)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	if (dev->params.reference_freq_Hz < ADF4351_REFIN_MIN_HZ || dev->params.reference_freq_Hz > ADF4351_REFIN_MAX_HZ)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	if (dev->params.reference_div_factor < ADF4351_R_COUNTER_MIN || dev->params.reference_div_factor > ADF4351_R_COUNTER_MAX)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	if (dev->params.channel_spacing_Hz < 1)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	const uint8_t ref_mul = dev->params.reference_mul2_enable ? 2 : 1;
	const uint8_t ref_div = dev->params.reference_div2_enable ? 2 : 1;

	const uint32_t min_int_div = (freq_Hz > ADF4351_45_PRESCALER_MAX_HZ) ? 75 : 23;

	dev->vco_Hz = freq_Hz;
	while (dev->vco_Hz < ADF4351_VCO_MIN_HZ)
	{
		dev->vco_Hz <<= 1;
		if (++dev->output_divider > ADF4351_OUT_DIV_MAX)
		{
			dev->output_divider = 0;
			dev->vco_Hz = 0;
			adf4351_set_regs(dev, false, true);
			return 0;
		}
	}

	if (!dev->params.as_near_as_possible)
	{
		dev->pfd_freq_Hz = (dev->params.reference_freq_Hz * ref_mul) / (dev->params.reference_div_factor * ref_div);
		if (dev->pfd_freq_Hz > ADF4351_PFD_FRAC_MAX_HZ)
		{
			dev->pfd_freq_Hz = 0;
			adf4351_set_regs(dev, false, true);
			return 0;
		}

		dev->mod = dev->pfd_freq_Hz / dev->params.channel_spacing_Hz;
		if (dev->mod > ADF4351_MODULUS_MAX)
		{
			dev->mod = 0;
			adf4351_set_regs(dev, false, true);
			return 0;
		}

		dev->int_div  =   dev->vco_Hz             / dev->pfd_freq_Hz;
		dev->frac_div = ((dev->vco_Hz * dev->mod) / dev->pfd_freq_Hz) - ((uint32_t)dev->int_div * dev->mod);
	}
	else
	{
		const uint32_t ref_in       = dev->params.reference_freq_Hz;
		uint16_t mod                = 0;
		uint16_t r_counter          = dev->params.reference_div_factor - 1;
		uint32_t channel_spacing_Hz = dev->params.channel_spacing_Hz;
		uint32_t pfd_freq_Hz;

		do {
			do {
				do {
					// increase the R-counter value until the PFD frequency is less than the allowed maximum
					do pfd_freq_Hz = (ref_in * ref_mul) / (++r_counter * ref_div);
					while (pfd_freq_Hz > ADF4351_PFD_FRAC_MAX_HZ);

					mod = pfd_freq_Hz / channel_spacing_Hz;

					if (r_counter > ADF4351_R_COUNTER_MAX)
					{	// try a larger channel spacing
						channel_spacing_Hz++;
						r_counter = 0;
					}
				} while (mod > ADF4351_MODULUS_MAX && r_counter > 0);
			} while (r_counter == 0);

			const uint64_t tmp      = ((dev->vco_Hz * mod) + (pfd_freq_Hz > 1)) / pfd_freq_Hz;
			dev->frac_div           = tmp % mod;
			dev->int_div            = tmp / mod;
			dev->pfd_freq_Hz        = pfd_freq_Hz;
			dev->mod                = mod;
			dev->r_counter          = r_counter;
			dev->channel_spacing_Hz = channel_spacing_Hz;

		} while (dev->int_div < min_int_div);

		if (dev->mod == 0)
		{
			dev->frac_div = 0;
			dev->output_divider = 0;
			dev->vco_Hz = 0;
			adf4351_set_regs(dev, false, true);
			return 0;
		}
	}

	// GCD (Greatest Common Divisor)
	if (dev->params.gcd_enable && dev->frac_div > 0 && dev->mod > 0)
	{
		//const uint32_t _gcd = gcd(dev->mod, dev->frac_div);
		//if (_gcd > 1)
		//{
		//	dev->frac_div /= _gcd;
		//	dev->mod      /= _gcd;
		//}

		uint32_t gcd = (dev->mod < dev->frac_div) ? dev->mod : dev->frac_div;
		while (gcd > 1 && ((dev->mod % gcd) || (dev->frac_div % gcd)))
			gcd--;
		if (gcd > 1)
		{
			dev->frac_div /= gcd;
			dev->mod      /= gcd;
		}
	}

	if (dev->params.low_spur_mode_enable && dev->mod > 0)
	{	// MOD must be >= 50 for low spur mode
		while (dev->mod < 50 && (dev->frac_div << 1) <= ADF4351_FRAC_MAX)
		{
			dev->frac_div <<= 1;
			dev->mod      <<= 1;
		}
	}

	const uint32_t band_select_max_Hz = (dev->params.band_select_clock_mode_high_enable) ? ADF4351_BANDSEL_HIGH_MAX_HZ : ADF4351_BANDSEL_LOW_MAX_HZ;
	dev->band_sel_div = dev->pfd_freq_Hz / band_select_max_Hz;
	if ((dev->pfd_freq_Hz % band_select_max_Hz) > (band_select_max_Hz / 2))
		dev->band_sel_div++;

	dev->frequency_Hz = adf4351_get_freq(dev);

	if (adf4351_set_regs(dev, (freq_Hz > 0) ? true : false, force) < 0)
	{
		adf4351_set_regs(dev, false, true);
		return 0;
	}

	return dev->frequency_Hz;
}

#if defined(ADF4351_LD_GPIO_Port) && defined(ADF4351_LD_Pin)
	bool adf4351_locked(t_adf4351_dev *dev, uint32_t timeout_us)
	{
		bool locked = false;

		if (dev == nullptr)
			return locked;

		locked = (HAL_GPIO_ReadPin(ADF4351_LD_GPIO_Port, ADF4351_LD_Pin) == GPIO_PIN_SET) ? true : false;	// true = locked

		if (timeout_us == 0)
			return locked;

		#if defined(DWT)

			ITM->LAR          = 0xC5ACCE55;                      // unlock the debug registers
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;      //
			DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;          // turn on the cycle counter
			DWT->CYCCNT       = 0;                               // reset cycle counter

			const uint32_t timeout_cycles = ((uint64_t)HAL_RCC_GetHCLKFreq() * timeout_us) / 1000000;

			const uint32_t lock_cycles    = ((uint64_t)HAL_RCC_GetHCLKFreq() * 500) / 1000000;      // VFO needs to stay locked for at least 500us

			uint32_t lock_cycle = 0;

			// wait for the VFO to lock - with timeout
			while (1)
			{
				const uint32_t cycle = DWT->CYCCNT;

				locked = (HAL_GPIO_ReadPin(ADF4351_LD_GPIO_Port, ADF4351_LD_Pin) == GPIO_PIN_SET) ? true : false;

				if (cycle >= timeout_cycles)
					break;                // timed out

				if (!locked)
				{	// VFO unlocked
					lock_cycle = 0;
					continue;
				}

				if (lock_cycle == 0)
					lock_cycle = cycle;  // start of lock

				if ((cycle - lock_cycle) >= lock_cycles)
					break;               // VFO has been locked for long enough
			}

		#else

			{	// wait for start of next tick
				const uint32_t tick = HAL_GetTick();
				while (HAL_GetTick() == tick);
			}

			const uint32_t ms = (timeout_us + 999) / 1000;

			const uint32_t start_tick = HAL_GetTick();

			uint32_t lock_tick = 0;

			// wait for the VFO to lock - with timeout
			while (1)
			{
				const uint32_t tick = HAL_GetTick();

				locked = (HAL_GPIO_ReadPin(ADF4351_LD_GPIO_Port, ADF4351_LD_Pin) == GPIO_PIN_SET) ? true : false;

				if (ms > 0 && (tick - start_tick) >= ms)
					break;               // timed out

				if (!locked)
				{	// VFO unlocked
					lock_tick = 0;
					continue;
				}

				if (lock_tick == 0)
					lock_tick = tick;    // start of lock

				if ((tick - lock_tick) >= 1)
					break;               // VFO has been locked for long enough
			}

		#endif

		return locked;
	}
#endif

int adf4351_init(t_adf4351_dev *dev)
{
	if (dev == nullptr)
		return -1;

	#if defined(ADF4351_CE_GPIO_Port) && defined(ADF4351_CE_Pin)
		HAL_GPIO_WritePin(ADF4351_CE_GPIO_Port,   ADF4351_CE_Pin,   GPIO_PIN_SET);
	#endif

	HAL_GPIO_WritePin(ADF4351_LE_GPIO_Port,   ADF4351_LE_Pin,   GPIO_PIN_SET);

	#if !defined(ADF4351_SPI_PORT)
		HAL_GPIO_WritePin(ADF4351_DATA_GPIO_Port, ADF4351_DATA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ADF4351_CLK_GPIO_Port,  ADF4351_CLK_Pin,  GPIO_PIN_RESET);
	#endif

	// settings are now valid
	dev->magic = ADF4351_MAGIC;

	if (adf4351_set_freq(dev, dev->params.frequency_Hz, true) == 0)
		return -2;

	return 0;	// OK
}

void adf4351_init_defaults(t_adf4351_dev *dev)
{
	if (dev == nullptr)
		return;

	dev->params.reference_freq_Hz                       = 25000000;               // reference input frequency
	dev->params.channel_spacing_Hz                      = 25000;                  // desired channel spacing
	dev->params.frequency_Hz                            = 0;                      // desired output frequency
	dev->params.reference_div_factor                    = 1;                      // reference input clock divider
	dev->params.reference_mul2_enable                   = false;                  // disable reference input multiple-by-2
	dev->params.reference_div2_enable                   = false;                  // disable reference input divide-by-2
	dev->params.gcd_enable                              = true;                   // enable GCD
	dev->params.phase_detector_polarity_positive_enable = true;                   // positive PD polarity
	dev->params.lock_detect_precision_6ns_enable        = false;                  // 10ns lock detector precision
	dev->params.lock_detect_function_integer_n_enable   = false;                  // fractional PLL lock detector mode
	dev->params.charge_pump_current                     = 7;                      // 2.5mA
	dev->params.muxout_select                           = 6;                      // mux output = digital lock detect state
	dev->params.low_spur_mode_enable                    = false;                  // lower noise mode
	dev->params.cycle_slip_reduction_enable             = false;                  //
	dev->params.charge_cancellation_enable              = false;                  //
	dev->params.anti_backlash_3ns_enable                = false;                  // 6ns (FRAC-N)
	dev->params.band_select_clock_mode_high_enable      = false;                  // band select low mode
	dev->params.clock_divider_value                     = 150;                    // clock divider value
	dev->params.clock_divider_mode                      = 0;                      // disable clock divider
	dev->params.mute_till_lock_enable                   = false;                  // disable output muting when VCO is unlocked
	dev->params.output_power                            = 4;                      // main output +5dBm
	dev->params.aux_output_power                        = 0;                      // disable AUX output
	dev->params.aux_output_select                       = true;							// AUX output VCO fundamental
	dev->params.as_near_as_possible                     = false;                  // compute values to get as close as possible to the desired frequency
}
