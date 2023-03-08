#ifndef _CONFIG_H
#define _CONFIG_H
#include "main.h"
#include "events.h"

#include "esp32_interface.h"
#include "stdint.h"
#include "string.h"
//#include "msg.h"


void Config_Handler(spi_cmd_t *  cmd);
uint8_t Config_Set_Adc_channels(uint8_t channels);
uint8_t Config_Set_Resolution(uint8_t resolution);
uint8_t Config_Set_Sample_freq(uint8_t sampleFreq);
uint8_t Config_Set_Time(uint32_t epoch);
#endif


