//#include "esp_system.h"
#include "stdint.h"
#include "iirfilter.h"
//#include "settings.h"

// Original coefficients from https://tecnionnl.sharepoint.com/:x:/s/uberlogger/EeEoN_zLy7BHslnFgKYobd4BH9o46vYH16z9PU2SE_CJCw?e=e9fFJc
// 0.085849253
// 0.164328412
// 0.324768093
// 0.544061872
// 0.792120424
// 0.956786082
//#define TAG_IIR "IIR"
#define FIXEDPT_WBITS 16
#include "fixedptc.h"


#define NUM_COEFFICIENTS 8   // Filter length
#define NUM_ADC_CHANNELS 8

//const int64_t c[NUM_COEFFICIENTS] = {8584925, 16432841, 32476809, 54406187, 79212042, 95678608, 95678608, 95678608};      // mulitplied with 100000000

 fixedpt cfl[NUM_COEFFICIENTS];

//int64_t x_state[NUM_ADC_CHANNELS];
uint16_t y_state[NUM_ADC_CHANNELS] = {0};
uint8_t coeff_index = 0;

//fixedpt cfp, input_fp, output_fp;
uint16_t cfp;

uint16_t round_value(uint32_t val) {
    return (val + 32768) >> 16;  // Add half of 2^16 to round, then shift
}

uint16_t divRoundClosest(const uint32_t n)
{
  const uint16_t d = 32768;
  return ((n < 0) == (d < 0)) ? ((n + d/2)/d) : ((n - d/2)/d);
}

uint32_t temp[NUM_ADC_CHANNELS] = {0};

void iir_filter(uint16_t * input, uint16_t * output, uint8_t channel)
{
    // Based on the factor, we need to pick the correct coefficients 

    // Multiply and accumulate
    // ESP_LOGI(TAG_IIR, "input: %ld, coeff: %lld, y_state: %lld", input, c[channel][coeff_index], y_state[channel]);
//    y_state[channel] = ((c[coeff_index] * (int64_t)*input ) + ((100000000LL-c[coeff_index]) * y_state[channel]))/ 100000000LL;
//     input_fp = fixedpt_fromint(*input);
//     output_fp = fixedpt_fromint(y_state[channel]);
	temp[channel] = temp[channel] + cfp * (*input - *output);
//	y_state[channel] = divRoundClosest(temp[channel]);
	y_state[channel] = round_value(temp[channel]);

	*output = y_state[channel];

//     y_state[channel] += fixedpt_toint(fixedpt_mul(cfp, input_fp-output_fp));
    // the factor 1000000 is used 
//    *output = (uint16_t)(y_state[channel]);
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
	if (	sampleFreq >= ADC_SAMPLE_RATE_1Hz &&
			sampleFreq <= ADC_SAMPLE_RATE_250Hz)
	{
		coeff_index = sampleFreq;
		cfp = cfl[coeff_index];

		return 0;
	}

	return 1;

}

void iir_init()
{
	//{ 0.00184806, 0.003692705, 0.00920621, 0.018327665, 0.045191272, 0.088340294, 0.16887658, 0.370256345};
	cfl[0] = fixedpt_rconst(0.00184806);
	cfl[1] = fixedpt_rconst(0.003692705);
	cfl[2] = fixedpt_rconst(0.00920621);
	cfl[3] = fixedpt_rconst(0.018327665);
	cfl[4] = fixedpt_rconst(0.045191272);
	cfl[5] = fixedpt_rconst(0.088340294);
	cfl[6] = fixedpt_rconst(0.16887658);
	cfl[7] = fixedpt_rconst(0.370256345);

}





