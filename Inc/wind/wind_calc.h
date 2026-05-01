#ifndef WIND_CALC_H
#define WIND_CALC_H

#include <stdint.h>
#include <stdbool.h> // Required to use the 'bool' data type
#include "wind.h"

/* --- MODULE INITIALIZATION & BACKGROUND TASKS --- */
// Inizializza il modulo e carica i dati dalla EEPROM al boot
void wind_calc_init(void);

// Task continuo per l'auto-apprendimento della scia aerodinamica
void wind_calc_background_task(void);


/* --- WIND FUNCTION DECLARATIONS --- */

// Processes wind waveform data for a specific channel and direction
void processWindWaveform(uint8_t channel, uint8_t direction);

// Calculates wind speed. Returns 'true' if the calculation is valid, 'false' if discarded.
bool calculate_wind(int16_t *x_cmps, int16_t *y_cmps);

// Stores a filtered wind sample
void store_wind_sample(int16_t x_cmps, int16_t y_cmps);

// Retrieves calculated parameters (Scommentata, ora è attiva nel .c)
void get_wind_parameters(uint16_t *pAvg_dmps, uint16_t *pGust_dmps, uint16_t *pAngle_deg);

// Prints debug data to the serial interface
void print_wind_calc_debug(void);


/* --- PHASE CALIBRATION FUNCTION DECLARATIONS (Tasto CAL) --- */
void begin_calibration(void);
void store_calibration_sample(void);
bool maybe_end_calibration(void);
void recallCalibration(void);


/* --- WAKE CALIBRATION FUNCTION DECLARATIONS (Aerodinamica) --- */
void begin_wake_calibration(void);
bool end_wake_calibration(void);
void recallWakeCalibration(void);

#endif // WIND_CALC_H
