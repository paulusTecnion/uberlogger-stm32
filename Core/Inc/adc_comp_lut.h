#ifndef _ADC_COMP_LUT_H
#define _ADC_COMP_LUT_H
#include "stdint.h"
#include "esp32_interface.h"

#define Q 16
#define FIXEDPT_WBITS 16

typedef struct { int32_t x; int32_t y; } lut_t;


uint16_t adc_comp(lut_t * table, uint16_t *input);
uint8_t adc_set_lut(adc_channel_range_t range, adc_resolution_t resolution, uint8_t channel);
uint16_t interp( lut_t * c, uint16_t x, int n );

#endif
