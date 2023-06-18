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

#define NUM_COEFFICIENTS 8   // Filter length
#define NUM_ADC_CHANNELS 8
const int64_t c[NUM_COEFFICIENTS] = {8584925, 16432841, 32476809, 54406187, 79212042, 95678608, 95678608, 95678608};      // mulitplied with 100000000



//int64_t x_state[NUM_ADC_CHANNELS];
int64_t y_state[NUM_ADC_CHANNELS];
uint8_t coeff_index = 0;



void iir_filter(uint16_t * input, uint16_t * output, uint8_t channel)
{
    // Based on the factor, we need to pick the correct coefficients 

    // Multiply and accumulate
    // ESP_LOGI(TAG_IIR, "input: %ld, coeff: %lld, y_state: %lld", input, c[channel][coeff_index], y_state[channel]);
    y_state[channel] = ((c[coeff_index] * (int64_t)*input ) + ((100000000LL-c[coeff_index]) * y_state[channel]))/ 100000000LL;

    // the factor 1000000 is used 
    *output = (uint16_t)(y_state[channel]);
}

void iir_reset()
{
    // Clear state
    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        y_state[i] = 0;
    }

}

uint8_t iir_set_samplefreq(uint8_t sampleFreq)
{
	if (	sampleFreq >= ADC_SAMPLE_RATE_1Hz &&
			sampleFreq <= ADC_SAMPLE_RATE_250Hz)
	{
		coeff_index = sampleFreq;
		return 0;
	}

	return 1;

}






