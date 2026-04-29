#ifndef WIND_CALC_H
#define WIND_CALC_H

#include <stdint.h>
#include <stdbool.h> // Required to use the 'bool' data type
#include "wind.h"

/* --- WIND FUNCTION DECLARATIONS --- */

// Processes wind waveform data for a specific channel and direction
void processWindWaveform(uint8_t channel, uint8_t direction);

// Calculates wind speed. Returns 'true' if the calculation is valid, 'false' if discarded.
bool calculate_wind(int16_t *x_cmps, int16_t *y_cmps);

// Stores a filtered wind sample
void store_wind_sample(int16_t x_cmps, int16_t y_cmps);

// Retrieves calculated parameters (currently commented out in your original code)
// void get_wind_parameters(uint16_t* pAvg_dmps, uint16_t* pGust_dmps, uint16_t* pAngle_deg);

// Prints debug data to the serial interface
void print_wind_calc_debug();

/* --- CALIBRATION FUNCTION DECLARATIONS --- */

void begin_calibration();
void store_calibration_sample();
bool maybe_end_calibration();
void recallCalibration();

#endif // WIND_CALC_H
