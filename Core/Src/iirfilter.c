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
#define FIXEDPT_WBITS 17
#include "fixedptc.h"


#define NUM_COEFFICIENTS 8   // Filter length
#define NUM_ADC_CHANNELS 8

//const int64_t c[NUM_COEFFICIENTS] = {8584925, 16432841, 32476809, 54406187, 79212042, 95678608, 95678608, 95678608};      // mulitplied with 100000000

const float cfl[NUM_COEFFICIENTS] = { 0.00184806, 0.003692705, 0.00920621, 0.018327665, 0.045191272, 0.088340294, 0.16887658, 0.370256345};

//int64_t x_state[NUM_ADC_CHANNELS];
uint32_t y_state[NUM_ADC_CHANNELS];
uint8_t coeff_index = 0;

fixedpt cfp, input_fp, output_fp;
fixedpt x_fp, c_x_fp, diffx_fp, diffn_fp;



uint16_t interp( lut_t * c, uint16_t x, int n ){
    int i;


    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
        	//            int32_t diffx = x - c[i].x;
        	//            int32_t diffn = c[i+1].x - c[i].x;
        	diffx_fp = fixedpt_fromint(x - c[i].x);
        	diffn_fp = fixedpt_fromint(c[i+1].x - c[i].x);

//        	return (c[i].y << Q) + q_mul(( c[i+1].y - c[i].y ) << Q, q_div(diffx << Q, diffn << Q));
        	output_fp =  fixedpt_fromint(c[i].y) + fixedpt_mul(fixedpt_fromint(c[i+1].y - c[i].y), fixedpt_div(diffx_fp, diffn_fp));
        	return fixedpt_toint(output_fp);
        }
    }
    return x; // Not in range, just return the input
}



void iir_filter(uint16_t * input, uint16_t * output, uint8_t channel)
{
    // Based on the factor, we need to pick the correct coefficients 

    // Multiply and accumulate
    // ESP_LOGI(TAG_IIR, "input: %ld, coeff: %lld, y_state: %lld", input, c[channel][coeff_index], y_state[channel]);
//    y_state[channel] = ((c[coeff_index] * (int64_t)*input ) + ((100000000LL-c[coeff_index]) * y_state[channel]))/ 100000000LL;
     input_fp = fixedpt_fromint(*input);
     output_fp = fixedpt_fromint(y_state[channel]);


     y_state[channel] += fixedpt_toint(fixedpt_mul(cfp, input_fp-output_fp));
    // the factor 1000000 is used 
    *output = (uint16_t)(y_state[channel]);
}

void iir_reset()
{
    // Clear state
    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        y_state[i] = 0;
        input_fp = 0;
        output_fp = 0;
    }

}

uint8_t iir_set_samplefreq(uint8_t sampleFreq)
{
	if (	sampleFreq >= ADC_SAMPLE_RATE_1Hz &&
			sampleFreq <= ADC_SAMPLE_RATE_250Hz)
	{
		coeff_index = sampleFreq;
		cfp = fixedpt_rconst(cfl[coeff_index]);

		return 0;
	}

	return 1;

}






