/*
 * MIT License
 *
 * Copyright (c) 2025 Tecnion Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//#include "esp_system.h"
#include "stdint.h"
#include "iirfilter.h"
//#include "settings.h"


#define FIXEDPT_WBITS 16
#include "fixedptc.h"


#define NUM_COEFFICIENTS 8   // Filter length
#define NUM_ADC_CHANNELS 8

 fixedpt cfl[NUM_COEFFICIENTS];

//int64_t x_state[NUM_ADC_CHANNELS];
uint16_t y_state[NUM_ADC_CHANNELS] = {0};
uint8_t coeff_index = 0;

//fixedpt cfp, input_fp, output_fp;
uint16_t cfp;

uint16_t round_value(uint32_t val) {
    return (val + 32768) >> 16;  // Add half of 2^16 to round, then shift
}



uint32_t temp[NUM_ADC_CHANNELS] = {0};

void iir_filter(uint16_t * input, uint16_t * output, uint8_t channel)
{
	// Here we calculate the error in 32 bits. Then we add a fraction of the error to the output of signal. This way we smartly solve the limit cycle problem with iir filters.
	// See: https://dsp.stackexchange.com/questions/66171/single-pole-iir-filter-fixed-point-design
	temp[channel] = temp[channel] + cfp * (*input - *output);

	y_state[channel] = round_value(temp[channel]);

	*output = y_state[channel];

}

void iir_reset()
{
    // Clear state
    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        y_state[i] = 0;
//        input_fp = 0;
//        output_fp = 0;
    }

}

uint8_t iir_set_samplefreq(uint8_t sampleFreq)
{
//	if (	sampleFreq >= ADC_SAMPLE_RATE_EVERY_60S &&
	if (	sampleFreq >= ADC_SAMPLE_RATE_1Hz &&
			sampleFreq <= ADC_SAMPLE_RATE_250Hz)
	{
		// Only coefficients for 1 to 250 Hz are available. So make an offset.
		coeff_index = sampleFreq - ADC_SAMPLE_RATE_1Hz;
		cfp = cfl[coeff_index];

		return 0;
	}

	return 1;

}

void iir_init()
{
	// Original coefficients from https://tecnionnl.sharepoint.com/:x:/s/uberlogger/EeEoN_zLy7BHslnFgKYobd4BH9o46vYH16z9PU2SE_CJCw?e=e9fFJc
	//{ 0.00184806, 0.003692705, 0.00920621, 0.018327665, 0.045191272, 0.088340294, 0.16887658, 0.370256345};
//	cfl[0] = fixedpt_rconst(0.00184806); // taking this
//	cfl[1] = fixedpt_rconst(0.00184806);
//	cfl[2] = fixedpt_rconst(0.00184806);
//	cfl[3] = fixedpt_rconst(0.003692705);
//	cfl[4] = fixedpt_rconst(0.00920621);
//	cfl[5] = fixedpt_rconst(0.018327665);
//	cfl[6] = fixedpt_rconst(0.045191272);
//	cfl[7] = fixedpt_rconst(0.088340294);
//	cfl[8] = fixedpt_rconst(0.16887658);
//	cfl[9] = fixedpt_rconst(0.370256345);

	cfl[0] = fixedpt_rconst(0.00184806); // taking this
	cfl[1] = fixedpt_rconst(0.003692705);
	cfl[2] = fixedpt_rconst(0.00920621);
	cfl[3] = fixedpt_rconst(0.018327665);
	cfl[4] = fixedpt_rconst(0.045191272);
	cfl[5] = fixedpt_rconst(0.088340294);
	cfl[6] = fixedpt_rconst(0.16887658);
	cfl[7] = fixedpt_rconst(0.370256345);

}





