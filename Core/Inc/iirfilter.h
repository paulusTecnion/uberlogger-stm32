#ifndef __IIRFILTER_H
#define __IIRFILTER_H
#include "stdint.h"
#include "esp32_interface.h"



/// @brief 
/// @param input 
/// @return 
void  iir_filter(uint16_t * input, uint16_t * output, uint8_t channel);
uint8_t iir_set_samplefreq(uint8_t sampleFreq);
void iir_reset();


#endif
