#ifndef WIND_H
#define WIND_H

#include <stdint.h>
#include <stdbool.h>
#include "debug.h"

/* --- Debugging Macros --- */

#ifdef DEBUG_WIND
    #define WIND_PRINT(...) do { debug_print(__VA_ARGS__); } while (0)
#else
    #define WIND_PRINT(...) do {} while (0)
#endif

#ifdef DEBUG_WIND_TIME
    // FIX: Corrected variadic macro syntax to properly handle code blocks
    #define ON_WIND_TIME(...) do { __VA_ARGS__; } while (0)
    #define WIND_TIME_PRINT(...) WIND_PRINT(__VA_ARGS__)
#else
    #define ON_WIND_TIME(...) do {} while (0)
    #define WIND_TIME_PRINT(...) do {} while(0)
#endif

/* --- Type Definitions --- */

typedef enum
{
    N = 0, 
    S = 1,
    E = 2,
    W = 3
} transducer_t;

/* --- External Hardware State --- */
// These variables are defined in wind.c and used by the calculation engine

extern const uint8_t g_channel_transducers[6][2];
extern volatile uint16_t* g_wind_measurement;
extern uint8_t g_windRingCounts[6];
extern uint32_t g_signalPowers[6][2];
extern bool g_wind_init_required;

/* --- Main API Functions --- */

/**
 * @brief Processes the wind state machine. Handles timing and triggers sampling.
 * @return true if a measurement was performed.
 */
bool process_wind(void);

/**
 * @brief Initializes the wind sensor hardware and buffers.
 */
void initWind(void);

/**
 * @brief Prints raw ADC buffer data for debugging.
 */
void printWindDebug(void);

/**
 * @brief Retrieves the final processed wind data.
 * @param pAvg_dmps  Output for average speed (decimeters per second)
 * @param pGust_dmps Output for gust speed (decimeters per second)
 * @param pAngle_deg Output for wind direction (0-359 degrees)
 */
void get_wind_parameters(uint16_t* pAvg_dmps, uint16_t* pGust_dmps, uint16_t* pAngle_deg);

/**
 * @brief Starts the 200-sample manual calibration sequence (Zero Wind).
 */
void calibrateWind(void);

#endif // WIND_H
