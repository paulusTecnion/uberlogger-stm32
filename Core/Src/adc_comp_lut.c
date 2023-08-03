#include "adc_comp_lut.h"
#include "fixedptc.h"

#define ADC_LUT_SIZE 4


fixedpt x_fp, c_x_fp, diffx_fp, diffn_fp, output_fp_adc;

lut_t ADC_LUT_16_BIT[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {65534, 65534},
    {65534, 65534},
    {65534, 65534},
    {65534, 65534},
  };

lut_t ADC_LUT_12_BIT[ADC_LUT_SIZE] = { // LUT is not in Q notation, is converted in the interpolation function
    {4095, 4095},
    {4095, 4095},
    {4095, 4095},
    {4095, 4095},
  };


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
        	output_fp_adc =  fixedpt_fromint(c[i].y) + fixedpt_mul(fixedpt_fromint(c[i+1].y - c[i].y), fixedpt_div(diffx_fp, diffn_fp));
        	return fixedpt_toint(output_fp_adc);
        }
    }
    return x; // Not in range, just return the input
}

uint16_t adc_comp_12b(uint16_t *input)
{
	return interp(ADC_LUT_12_BIT, *input, ADC_LUT_SIZE);
}

uint16_t adc_comp_16b(uint16_t *input)
{
	return interp(ADC_LUT_16_BIT, *input, ADC_LUT_SIZE);
}



