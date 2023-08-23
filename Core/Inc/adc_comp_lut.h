#ifndef _ADC_COMP_LUT_H
#define _ADC_COMP_LUT_H
#include "stdint.h"
typedef struct { int32_t x; int32_t y; } lut_t;

uint16_t adc_comp_12b(uint16_t *input);
uint16_t adc_comp_16b(uint16_t *input);
uint16_t interp( lut_t * c, uint16_t x, int n );

#endif
