#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "debug.h"

extern const uint8_t radio_config[];
extern const uint8_t radio_freq_config[];
extern const uint8_t radio_data_rate_bank[];
extern const uint8_t radio_CUS_TX8_9_Values[];
extern uint8_t g_frequencySelector;

void init_radio();
void test_radio();
void configure_radio(bool first_init,uint16_t frequency_selector);
void radio_transmit(void* data, uint8_t len);
bool processRadio();

#define RADIO_TX_PERIOD 2.0

#ifdef DEBUG_RADIO
#define RADIO_PRINT(...) do { debug_print(__VA_ARGS__); } while (0)
#else
#define RADIO_PRINT(...) do {} while (0)
#endif
