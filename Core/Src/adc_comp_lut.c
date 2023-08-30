#include "adc_comp_lut.h"
#include "fixedptc.h"
#include "esp32_interface.h"

#define ADC_LUT_SIZE 7


fixedpt x_fp, c_x_fp, diffx_fp, diffn_fp, output_fp_adc;

lut_t ADC_LUT_16_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {145, 367},
    {10816, 11127},
    {27980, 28434},
    {32281, 32768},
	{36814, 37102},
    {53952, 54372},
	{64666, 65169}
  };

lut_t ADC_LUT_16_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {247, 459},
    {16897, 17256},
    {29191, 29659},
    {32280, 32768},
	{35595, 35877},
	{47906, 48273},
	{64595, 65077}
  };

lut_t ADC_LUT_12_BIT_10V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {17, 23},
    {692, 698},
    {1773, 1778},
    {2045, 2048},
	{2316, 2318},
	{3400, 3399},
	{4077, 4073}
  };

lut_t ADC_LUT_12_BIT_60V[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {24, 29},
    {1074, 1078},
    {1883, 1886},
    {2045, 2048},
	{2207, 2210},
	{3020, 3018},
	{4075, 4067}
  };

lut_t * active_lut_table[NUM_ADC_CHANNELS] =
		{ADC_LUT_12_BIT_10V};

uint16_t interp( lut_t * c, uint16_t x, int n ){
    int i;


    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
        	            int32_t diffx = x - c[i].x;
        	            int32_t diffn = c[i+1].x - c[i].x;
        	            int32_t result = 0;
//        	diffx_fp = fixedpt_fromint(x - c[i].x);
//        	diffn_fp = fixedpt_fromint(c[i+1].x - c[i].x);

        	result =  (c[i].y << Q) + q_mul(( c[i+1].y - c[i].y ) << Q, q_div(diffx << Q, diffn << Q));
        	return (result >> Q);
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




