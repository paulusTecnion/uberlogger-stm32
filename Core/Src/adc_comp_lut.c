#include "adc_comp_lut.h"
//#include "fixedptc.h"
#include "esp32_interface.h"

#define ADC_LUT_SIZE 7
#define Q 15

int32_t q_mul(int32_t a, int32_t b) {
    int64_t result;

    result = (int64_t)a * (int64_t)b;
    result += (1 << (Q - 1)); // correction of rounding

    return result >> Q;
}

int32_t q_div(int32_t a, int32_t b) {
    int64_t result;

    result = (int64_t)a << Q;
    // no rounding applied

    return (int32_t)(result / b);
}




/* Calibration Bas */
//lut_t ADC_LUT_16_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {145, 367},
//    {10816, 11124},
//    {27980, 28427},
//    {32281, 32760},
//	{36814, 37093},
//    {53952, 54359},
//	{64666, 65153}
//  };
//
//
//lut_t ADC_LUT_16_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {247, 459},
//    {16897, 17252},
//    {29191, 29652},
//    {32280, 32760},
//	{35595, 35868},
//	{47906, 48261},
//	{64595, 65061}
//  };
//
//lut_t ADC_LUT_12_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {17, 23},
//    {692, 698},
//    {1773, 1778},
//    {2045, 2048},
//	{2316, 2318},
//	{3400, 3399},
//	{4077, 4073}
//  };
//
//lut_t ADC_LUT_12_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {24, 29},
//    {1074, 1078},
//    {1883, 1886},
//    {2045, 2048},
//	{2207, 2210},
//	{3020, 3018},
//	{4075, 4067}
//  };

/* Calibration R02 */
//
//lut_t ADC_LUT_16_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {14, 367},
//    {10775, 11124},
//    {27980, 28427},
//    {32280, 32760},
//	{36825, 37093},
//    {54030, 54359},
//	{64780, 65153}
//  };
//
//
//lut_t ADC_LUT_16_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {96, 459},
//    {16833, 17252},
//    {29191, 29652},
//    {32285, 32760},
//	{35600, 35868},
//	{47970, 48261},
//	{64739, 65061}
//  };
//
//lut_t ADC_LUT_12_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {8, 23},
//    {688, 698},
//    {1776, 1778},
//    {2049, 2048},
//	{2321, 2318},
//	{3410, 3399},
//	{4090, 4073}
//  };
//
//lut_t ADC_LUT_12_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
//    {10, 29},
//    {1069, 1078},
//    {1883, 1886},
//    {2049, 2048},
//	{2211, 2210},
//	{3026, 3018},
//	{4084, 4067}
//  };

/* Calibration R04 */

lut_t ADC_LUT_16_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {365, 367},
    {11005, 11124},
    {28020, 28427},
    {32245, 32760},
	{36748, 37093},
    {53751, 54359},
	{64376, 65153}
  };


lut_t ADC_LUT_16_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {485, 459},
    {17002, 17252},
    {29198, 29652},
    {32250, 32760},
	{35548, 35868},
	{47768, 48261},
	{64334, 65061}
  };

lut_t ADC_LUT_12_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {31, 23},
    {703, 698},
    {1778, 1778},
    {2046, 2048},
	{2318, 2318},
	{3394, 3399},
	{4066, 4073}
  };

lut_t ADC_LUT_12_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {24, 29},
    {1074, 1078},
    {1883, 1886},
    {2048, 2048},
	{2210, 2210},
	{3015, 3018},
	{4060, 4067}
  };






lut_t * active_lut_table[NUM_ADC_CHANNELS] =
		{ADC_LUT_12_BIT_10V};

uint16_t interp( lut_t * c, uint16_t x, int n ){
    int i;

    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
        	            int32_t diffx = (int32_t)x - c[i].x;
        	            int32_t diffn = c[i+1].x - c[i].x;
        	            int32_t result = 0;
//        	diffx_fp = fixedpt_fromint(x - c[i].x);
//        	diffn_fp = fixedpt_fromint(c[i+1].x - c[i].x);

        	result =  (c[i].y << Q) + q_mul(( c[i+1].y - c[i].y ) << Q, q_div(diffx << Q, diffn << Q));
        	result = (result >> Q);
        	return (uint16_t)result;
//        	output_fp_adc =  fixedpt_fromint(c[i].y) + fixedpt_mul(fixedpt_fromint(c[i+1].y - c[i].y), fixedpt_div(diffx_fp, diffn_fp));
//        	return fixedpt_toint(output_fp_adc);
        }
    }
    return x; // Not in range, just return the input
}

uint16_t adc_comp(lut_t * table, uint16_t *input)
{
	return interp(table, *input, ADC_LUT_SIZE);
}

uint8_t adc_set_lut(adc_channel_range_t range, adc_resolution_t resolution, uint8_t channel)
{
	if (resolution == ADC_12_BITS)
	{
		// if & operation results in true, then this is 60V
		if (range & (1<<channel))
		{
			active_lut_table[channel] = ADC_LUT_12_BIT_60V;
		} else {
			active_lut_table[channel] = ADC_LUT_12_BIT_10V;
		}

	} else if (resolution == ADC_16_BITS){
		if (range & (1<<channel))
		{
			active_lut_table[channel] = ADC_LUT_16_BIT_60V;
		} else {
			active_lut_table[channel] = ADC_LUT_16_BIT_10V;
		}
	}

	return 0;
}




