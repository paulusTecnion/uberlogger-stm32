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
uint8_t Config_set_range(adc_channel_range_t range);
uint8_t Config_Set_Resolution(uint8_t resolution);
uint8_t Config_Set_Sample_freq(uint8_t sampleFreq);
uint8_t Config_Set_Time(uint32_t epoch);
uint8_t Config_set_logMode(uint8_t logtype, uint8_t data_lines_per_transaction);
uint8_t Config_set_triggerMode(uint8_t mode, uint8_t gpio);
uint8_t Config_set_debounceTime(uint32_t debounceTime);
#endif


