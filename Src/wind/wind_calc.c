/**
 * @file wind_calc.c
 * @brief Professional Ultrasonic Wind Calculation Module - Industrial Hurricane Grade
 * Final Implementation: 32-bit/64-bit path, trimmed-mean filtering, early rejection,
 * unified physics, safe EEPROM fallbacks, and deterministic math.
 */

#include "wind_calc.h"
#include "temperature.h"
#include "arm_math.h"
#include "fast_math_functions.h"
#include "stm32l1xx_hal_flash_ex.h"
#include "crc.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* --- 1. TYPE DEFINITIONS --- */
typedef int16_t q2_13, q1_14;
typedef int32_t q18_13, q17_14;

/* --- 2. CONSTANTS & CONFIG --- */
#define MEDIAN_WINDOW   7
#define MAX_ACCEL_CMPS  2500
#define qn_13_pi        25736
#define qn_14_one       16384
#define MAX_VALID_RESIDUAL 150000

/* Wake Correction Constants */
#define KAIMAL_A_K          0.1029f
#define KAIMAL_THETA_MAX    70.0f    

#define WAKE_DEFAULT_A          0.08f
#define WAKE_DEFAULT_SIGMA_DEG  17.0f
#define WAKE_A_Q15_DEFAULT      ((int16_t)(WAKE_DEFAULT_A * 32768.0f))       
#define WAKE_SIGMA_DEG10_DEFAULT ((int16_t)(WAKE_DEFAULT_SIGMA_DEG * 10.0f)) 

#define WAKE_CORRECTION_MAX     1.5f
#define WAKE_CORRECTION_MIN     1.0f

/* EEPROM addresses */
#define PHASE_CALIB_ADDRESS     0xA4   
#define WAKE_CALIB_ADDRESS      0xB4   

/* Wake calibration binning */
#define WAKE_BINS               36     
#define WAKE_CAL_MIN_PER_BIN    20     
#define WAKE_CAL_MIN_COVERAGE   30     
#define WAKE_CAL_MIN_SPEED      100    

/* --- 3. STATIC VARIABLES & STATE --- */
static int16_t rawBufferX[MEDIAN_WINDOW];
static int16_t rawBufferY[MEDIAN_WINDOW];
static int16_t lastFilteredX = 0, lastFilteredY = 0;
static uint8_t medianIdx     = 0;
static bool    bufferFull    = false;
static bool    firstSampleReceived = false;

static bool     s_isCalibrating    = false;
static uint16_t s_calibrationCount = 0;

const uint8_t VectorSumSize = 4;
int16_t  lastSampleX = 0, lastSampleY = 0;
static int32_t  spdVectorSumX, spdVectorSumY;
static int32_t  dirVectorSumX, dirVectorSumY;
static uint32_t spdSum  = 0;
static uint16_t maxSpd  = 0;
uint16_t spdVectorSumCnt = 0, dirVectorSumCnt = 0, spdSumCnt = 0;

/* Phase calibration */
const uint16_t calibrationAddress = PHASE_CALIB_ADDRESS;
q2_13 s_signalPhases[6][2];

struct __attribute__((packed, aligned(2))) calibrationStruct {
    q2_13   phases[6];
    uint8_t crc;
};
struct calibrationStruct s_calibrationData;
q18_13 s_calibrationPhaseSums[6];

/* Wake calibration */
struct __attribute__((packed, aligned(2))) wakeCalibStruct {
    int16_t A_q15;          
    int16_t sigma_deg10;    
    uint8_t crc;
};
static struct wakeCalibStruct s_wakeCalib;

static bool     s_isWakeCalibrating = false;
static uint32_t s_wakeSumSpeed[WAKE_BINS];  
static uint16_t s_wakeCntBin[WAKE_BINS];    

/* --- 4. PROTOTYPES --- */
uint32_t FastIntSqrt(uint32_t value);
uint32_t FastIntSqrt64(uint64_t value);
void     storeCalibration(void);
void     storeWakeCalibration(void);
void     recallWakeCalibration(void);
void     WriteEEPROM(uint16_t biasAddress, void *data, uint16_t len);
void     readEEPROM(uint16_t biasAddress, void *data, uint16_t len);
q18_13   normalise_angle(q18_13 angle);
int32_t  get_speed_cmps_32(uint8_t channel, int32_t timeDiff_ns);
static void apply_wake_correction(int16_t *x, int16_t *y);

/* --- 5. SAFE MATH HELPERS & UNIFIED PHYSICS --- */

static q15_t safe_scale_to_q15(int32_t v, int8_t shift) {
    if (v == 0) return 0;
    int64_t  mag    = (v < 0) ? -(int64_t)v : (int64_t)v;
    uint64_t scaled = (shift >= 0) ? ((uint64_t)mag << shift) : ((uint64_t)mag >> (-shift));
    if (scaled > 32767ULL) scaled = 32767ULL;
    return (v < 0) ? -(q15_t)scaled : (q15_t)scaled;
}

q18_13 normalise_angle(q18_13 angle) {
    int32_t period = 2 * qn_13_pi;
    angle = ((angle + qn_13_pi) % period + period) % period - qn_13_pi;
    return angle;
}

/**
 * @brief Centralized speed of sound squared calculation based on temperature.
 * v_s = 331.5 m/s @ 0°C. Temperature factor: 0.61 * T.
 */
static int32_t get_vs_squared(void) {
    int32_t v_s = 3315 + (g_tempMeasurement * 61) / 100;
    return v_s * v_s;
}

/* --- 6. MATH TABLES & VECTORS --- */
static int16_t sin24_5[49] = {
    0,519,1005,1425,1751,1963,2047,1997,1816,1516,1117,645,131,
    -391,-889,-1328,-1680,-1922,-2039,-2022,-1873,-1601,-1225,-769,
    -262,262,769,1225,1601,1873,2022,2039,1922,1680,1328,889,391,
    -131,-645,-1117,-1516,-1816,-1997,-2047,-1963,-1751,-1425,-1005,-519
};
static int16_t cos24_5[49] = {
    2048,1981,1784,1471,1062,583,66,-456,-947,-1377,-1716,-1944,
    -2044,-2010,-1845,-1559,-1172,-707,-197,327,829,1277,1641,1898,
    2031,2031,1898,1641,1277,829,327,-197,-707,-1172,-1559,-1845,
    -2010,-2044,-1944,-1716,-1377,-947,-456,66,583,1062,1471,1784,1981
};
static int16_t window[245] = {
    1,3,6,11,17,24,33,43,54,66,80,95,112,130,148,169,190,213,236,261,
    288,315,343,373,404,435,468,502,537,572,609,647,685,725,765,806,848,
    891,935,979,1024,1070,1116,1163,1210,1258,1307,1356,1405,1455,1505,
    1556,1607,1658,1710,1761,1813,1865,1917,1970,2022,2074,2126,2179,
    2231,2283,2335,2283,2231,2179,2126,2074,2022,1970,1917,1865,1813,
    1761,1710,1658,1607,1556,1505,1455,1405,1356,1307,1258,1210,1163,
    1116,1070,1024,979,935,891,848,806,765,725,685,647,609,572,537,502,
    468,435,404,373,343,315,288,261,236,213,190,169,148,130,112,95,80,
    66,54,43,33,24,17,11,6,3,1
};

q1_14 s_radiusVectors[6][2] = {
    { 11585, -11585 }, { -11585, -11585 }, { 0, -16384 },
    { -16384, 0 },     { -11585, -11585 }, { 11585, -11585 }
};
q1_14 s_tangentVectors[6][2] = {
    { 11585, 11585 }, { 11585, -11585 }, { 16384, 0 },
    { 0, 16384 },     { 11585, -11585 }, { 11585, 11585 }
};

/* ============================================================
 * LAYER 1+2: WAKE CORRECTION
 * ============================================================ */
static void apply_wake_correction(int16_t *x, int16_t *y)
{
    if (*x == 0 && *y == 0) return;

    /* SAFEGUARD: Prevent division by zero if EEPROM is corrupted or uninitialized.
       Sigma must be at least 5.0 degrees (50). Amplitude must be positive. */
    if (s_wakeCalib.sigma_deg10 < 50 || s_wakeCalib.A_q15 <= 0) {
        s_wakeCalib.sigma_deg10 = WAKE_SIGMA_DEG10_DEFAULT;
        s_wakeCalib.A_q15       = WAKE_A_Q15_DEFAULT;
    }

    float fx  = (float)*x;
    float fy  = (float)*y;
    float mag = sqrtf(fx * fx + fy * fy);
    if (mag < 1.0f) return;

    /* Wind direction in degrees [0, 360) */
    float wind_deg = atan2f(fy, fx) * (180.0f / (float)M_PI);
    if (wind_deg < 0.0f) wind_deg += 360.0f;

    /* LAYER 1: Kaimal (1979) Transducer Shadow Correction */
    const float path_axes_deg[4] = { 45.0f, 135.0f, 90.0f, 0.0f };
    float kaimal_attenuation = 0.0f;

    for (int p = 0; p < 4; p++) {
        float dtheta = fabsf(wind_deg - path_axes_deg[p]);
        if (dtheta > 180.0f) dtheta = 360.0f - dtheta;
        if (dtheta > 90.0f)  dtheta = 180.0f - dtheta;

        if (dtheta < KAIMAL_THETA_MAX) {
            kaimal_attenuation += KAIMAL_A_K * (1.0f - dtheta / KAIMAL_THETA_MAX);
        }
    }
    float k_kaimal = 1.0f / (1.0f - kaimal_attenuation / 4.0f);

    /* LAYER 2: Wyngaard & Zhang (1985) Structural Wake Correction */
    float A     = (float)s_wakeCalib.A_q15    / 32768.0f;
    float sigma = (float)s_wakeCalib.sigma_deg10 / 10.0f;
    float two_sigma_sq = 2.0f * sigma * sigma;

    const float strut_dirs[4] = { 0.0f, 90.0f, 180.0f, 270.0f };
    float gauss_sum = 0.0f;

    for (int s = 0; s < 4; s++) {
        float dtheta = wind_deg - strut_dirs[s];
        if (dtheta >  180.0f) dtheta -= 360.0f;
        if (dtheta < -180.0f) dtheta += 360.0f;
        gauss_sum += expf(-(dtheta * dtheta) / two_sigma_sq);
    }

    float denominator = 1.0f - A * gauss_sum;
    if (denominator < 0.5f) denominator = 0.5f;
    float k_gaussian = 1.0f / denominator;

    /* COMBINED CORRECTION */
    float correction = k_kaimal * k_gaussian;

    if (correction < WAKE_CORRECTION_MIN) correction = WAKE_CORRECTION_MIN;
    if (correction > WAKE_CORRECTION_MAX) correction = WAKE_CORRECTION_MAX;

    float new_mag = mag * correction;

    *x = (int16_t)(fx / mag * new_mag);
    *y = (int16_t)(fy / mag * new_mag);
}

/* --- 7. CORE LOGIC --- */

static int16_t get_trimmed_mean_7(int16_t *data) {
    int16_t temp[MEDIAN_WINDOW];
    memcpy(temp, data, MEDIAN_WINDOW * sizeof(int16_t));
    for (uint8_t i = 1; i < MEDIAN_WINDOW; i++) {
        int16_t key = temp[i];
        int8_t  j   = (int8_t)i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    int32_t sum = 0;
    for (uint8_t i = 1; i < (MEDIAN_WINDOW - 1); i++) {
        sum += temp[i];
    }
    return (int16_t)(sum / (MEDIAN_WINDOW - 2));
}

void processWindWaveform(uint8_t channel, uint8_t direction) {
    const uint8_t sampleMax = 245;
    int32_t sum = 0;
    for (uint8_t i = 0; i < sampleMax; i++) sum += (int32_t)g_wind_measurement[i];
    int32_t avg = sum / (int32_t)sampleMax;
    int32_t cosSum = 0, sinSum = 0;

    for (uint8_t i = 0; i < sampleMax; i++) {
        int32_t val = ((int32_t)g_wind_measurement[i] - avg) * (int32_t)window[i] >> 4;
        cosSum += val * (int32_t)cos24_5[i % 49] >> 8;
        sinSum += val * (int32_t)sin24_5[i % 49] >> 8;
    }

    uint32_t uAbsCos = (cosSum < 0) ? (uint32_t)-cosSum : (uint32_t)cosSum;
    uint32_t uAbsSin = (sinSum < 0) ? (uint32_t)-sinSum : (uint32_t)sinSum;
    uint32_t maxVal  = (uAbsCos > uAbsSin) ? uAbsCos : uAbsSin;

    q15_t finalCos = 0, finalSin = 0;
    if (maxVal > 0) {
        int8_t shift = (int8_t)__builtin_clz(maxVal) - 17;
        finalCos = safe_scale_to_q15(cosSum, shift);
        finalSin = safe_scale_to_q15(sinSum, shift);
    }
    g_signalPowers[channel][direction] =
        (uint32_t)((int32_t)finalCos * finalCos + (int32_t)finalSin * finalSin);
    arm_atan2_q15(finalSin, finalCos, &s_signalPhases[channel][direction]);
}

bool calculate_wind(int16_t *x_cmps, int16_t *y_cmps) {
    q18_13 signalPhaseDiffs[6];
    for (uint8_t ch = 0; ch < 6; ch++) {
        signalPhaseDiffs[ch] = normalise_angle((q18_13)s_signalPhases[ch][0] - s_signalPhases[ch][1] - s_calibrationData.phases[ch]);
    }

    /* Use unified physics formula */
    int32_t vs_sq = get_vs_squared();
    int32_t k_short = vs_sq / 16000;
    int32_t k_long  = vs_sq / 35700;

    uint32_t best_res = 0xFFFFFFFF;
    int32_t best_x = 0, best_y = 0;

    for (int8_t ch2W = -3; ch2W <= 3; ch2W++) {
        for (int8_t ch3W = -3; ch3W <= 3; ch3W++) {
            for (int8_t ch04W = -1; ch04W <= 1; ch04W++) {
                for (int8_t ch15W = -1; ch15W <= 1; ch15W++) {

                    int32_t sum_x = 0, sum_y = 0;
                    int32_t current_rV[6][2];

                    for (uint8_t ch = 0; ch < 6; ch++) {
                        q18_13 pD = signalPhaseDiffs[ch];

                        if (ch == 2) pD += (q18_13)2 * qn_13_pi * ch2W;
                        else if (ch == 3) pD += (q18_13)2 * qn_13_pi * ch3W;
                        else if (ch == 0 || ch == 4) pD += (q18_13)2 * qn_13_pi * ch04W;
                        else if (ch == 1 || ch == 5) pD += (q18_13)2 * qn_13_pi * ch15W;

                        int32_t delay_ns = (pD * 3900) >> 13;
                        int32_t k = (ch == 2 || ch == 3) ? k_long : k_short;

                        int32_t spd = (int32_t)(((int64_t)k * delay_ns) / 2000);

                        int32_t rV_x = (spd * s_radiusVectors[ch][0]) >> 14;
                        int32_t rV_y = (spd * s_radiusVectors[ch][1]) >> 14;

                        current_rV[ch][0] = rV_x;
                        current_rV[ch][1] = rV_y;
                        sum_x += rV_x; sum_y += rV_y;
                    }

                    int32_t bX = sum_x / 3;
                    int32_t bY = sum_y / 3;
                    uint32_t res = 0;

                    for (uint8_t ch = 0; ch < 6; ch++) {
                        int32_t dx = bX - current_rV[ch][0];
                        int32_t dy = bY - current_rV[ch][1];
                        int64_t dot_t = ((int64_t)dx * s_tangentVectors[ch][0] + (int64_t)dy * s_tangentVectors[ch][1]) >> 14;
                        int64_t diff = (int64_t)dx * dx + (int64_t)dy * dy - (dot_t * dot_t);
                        if (diff < 0) diff = 0;
                        res += (uint32_t)(diff >> 2);
                        
                        /* Early rejection: CPU safeguard against impossible wraps */
                        if (res > best_res) break; 
                    }

                    if (res < best_res) {
                        best_res = res;
                        best_x = bX;
                        best_y = bY;
                    }
                }
            }
        }
    }

    if (best_res > MAX_VALID_RESIDUAL) {
        return false;
    }

    *x_cmps = (int16_t)(best_x > 32767 ? 32767 : (best_x < -32768 ? -32768 : best_x));
    *y_cmps = (int16_t)(best_y > 32767 ? 32767 : (best_y < -32768 ? -32768 : best_y));

    return true;
}

void store_wind_sample(int16_t x_cmps, int16_t y_cmps) {
    if (!firstSampleReceived) {
        for (int i = 0; i < MEDIAN_WINDOW; i++) {
            rawBufferX[i] = x_cmps;
            rawBufferY[i] = y_cmps;
        }
        lastFilteredX      = x_cmps;
        lastFilteredY      = y_cmps;
        firstSampleReceived = true;
        bufferFull          = true;
    }
    rawBufferX[medianIdx] = x_cmps;
    rawBufferY[medianIdx] = y_cmps;
    medianIdx = (medianIdx + 1) % MEDIAN_WINDOW;

    int16_t medX = get_trimmed_mean_7(rawBufferX);
    int16_t medY = get_trimmed_mean_7(rawBufferY);

    if (bufferFull) {
        int16_t dX = medX - lastFilteredX, dY = medY - lastFilteredY;
        if (abs(dX) > MAX_ACCEL_CMPS)
            medX = lastFilteredX + (dX > 0 ? MAX_ACCEL_CMPS : -MAX_ACCEL_CMPS);
        if (abs(dY) > MAX_ACCEL_CMPS)
            medY = lastFilteredY + (dY > 0 ? MAX_ACCEL_CMPS : -MAX_ACCEL_CMPS);
    }
    lastFilteredX = medX;
    lastFilteredY = medY;

    apply_wake_correction(&medX, &medY);

    lastSampleX = medX;
    lastSampleY = medY;

    if (s_isWakeCalibrating) {
        int16_t raw_x = lastFilteredX;
        int16_t raw_y = lastFilteredY;
        uint32_t spd = FastIntSqrt((uint32_t)((int32_t)raw_x * raw_x + (int32_t)raw_y * raw_y));
        
        if (spd >= WAKE_CAL_MIN_SPEED) {
            float dir_deg = atan2f((float)raw_y, (float)raw_x) * (180.0f / (float)M_PI);
            if (dir_deg < 0.0f) dir_deg += 360.0f;
            uint8_t bin = (uint8_t)(dir_deg / 10.0f) % WAKE_BINS;
            s_wakeSumSpeed[bin] += spd;
            s_wakeCntBin[bin]++;
        }
    }

    dirVectorSumX += (int32_t)medX; dirVectorSumY += (int32_t)medY; dirVectorSumCnt++;
    spdVectorSumX += (int32_t)medX; spdVectorSumY += (int32_t)medY; spdVectorSumCnt++;

    if (spdVectorSumCnt >= VectorSumSize) {
        int32_t avgX = spdVectorSumX / (int32_t)spdVectorSumCnt;
        int32_t avgY = spdVectorSumY / (int32_t)spdVectorSumCnt;
        
        /* 64-bit integer square root per calcoli sicuri senza perdita di dati */
        uint16_t spd = (uint16_t)FastIntSqrt64((uint64_t)((int64_t)avgX * avgX + (int64_t)avgY * avgY));
        
        spdSum += (uint32_t)spd;
        if (spd > maxSpd) maxSpd = spd;
        spdSumCnt++;
        spdVectorSumX = spdVectorSumY = spdVectorSumCnt = 0;
    }
}

void get_wind_parameters(uint16_t *pAvg_dmps, uint16_t *pGust_dmps, uint16_t *pAngle_deg) {
    *pAvg_dmps  = (spdSumCnt > 0)
                  ? (uint16_t)((spdSum + (spdSumCnt * 5)) / (spdSumCnt * 10))
                  : 0;
    *pGust_dmps = (uint16_t)(maxSpd / 10);

    if (dirVectorSumCnt > 0 && (dirVectorSumX != 0 || dirVectorSumY != 0)) {
        int32_t avgX  = dirVectorSumX / (int32_t)dirVectorSumCnt;
        int32_t avgY  = dirVectorSumY / (int32_t)dirVectorSumCnt;
        uint32_t maxV = (uint32_t)(abs(avgX) > abs(avgY) ? abs(avgX) : abs(avgY));
        int8_t shift  = (int8_t)__builtin_clz(maxV) - 17;
        q15_t fY = safe_scale_to_q15(avgY, shift);
        q15_t fX = safe_scale_to_q15(avgX, shift);
        q15_t angleQ;
        arm_atan2_q15(fY, fX, &angleQ);
        int32_t deg = (int32_t)(angleQ * 3667 + (1 << 18)) >> 19;
        while (deg < 0) deg += 360;
        *pAngle_deg = (uint16_t)(deg % 360);
    } else {
        *pAngle_deg = 0;
    }

    spdSum = spdSumCnt = dirVectorSumCnt = 0;
    dirVectorSumX = dirVectorSumY = maxSpd = 0;
}

/* Use unified physics formula */
int32_t get_speed_cmps_32(uint8_t channel, int32_t timeDiff_ns) {
    int32_t vs_sq = get_vs_squared();
    int32_t L     = (channel == 2 || channel == 3) ? 35700 : 16000;
    return (int32_t)(((int64_t)vs_sq / L) * timeDiff_ns / 2000);
}

uint32_t FastIntSqrt(uint32_t value) {
    if (!value) return 0;
    uint32_t xn = 1u << ((32 - __CLZ(value)) / 2);
    xn = (xn + value / xn) / 2;
    xn = (xn + value / xn) / 2;
    xn = (xn + value / xn) / 2;
    return xn;
}

/** * @brief Rock-solid binary numeral system square root for 64-bit integers.
 * Immune to overflow and precision loss. 
 */
uint32_t FastIntSqrt64(uint64_t value) {
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62; 

    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (value >= res + bit) {
            value -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)res;
}

/* ============================================================
 * PHASE CALIBRATION (existing, unchanged)
 * ============================================================ */

void begin_calibration(void) {
    memset(s_calibrationPhaseSums, 0, sizeof(s_calibrationPhaseSums));
    s_calibrationCount = 0;
    s_isCalibrating    = true;
}

void store_calibration_sample(void) {
    if (!s_isCalibrating) return;
    for (int i = 0; i < 6; i++)
        s_calibrationPhaseSums[i] += normalise_angle(
            s_signalPhases[i][0] - s_signalPhases[i][1]);
    s_calibrationCount++;
}

bool maybe_end_calibration(void) {
    if (!s_isCalibrating || s_calibrationCount < 200) return false;
    for (int i = 0; i < 6; i++)
        s_calibrationData.phases[i] =
            (int16_t)(s_calibrationPhaseSums[i] / (int32_t)s_calibrationCount);
    storeCalibration();
    s_isCalibrating    = false;
    s_calibrationCount = 0;
    return true;
}

void storeCalibration(void) {
    s_calibrationData.crc =
        crc8_dallas(&s_calibrationData, sizeof(s_calibrationData) - 1, 0xEE);
    WriteEEPROM(calibrationAddress, &s_calibrationData, sizeof(s_calibrationData));
}

void recallCalibration(void) {
    readEEPROM(calibrationAddress, &s_calibrationData, sizeof(s_calibrationData));
    if (crc8_dallas(&s_calibrationData, sizeof(s_calibrationData), 0xEE) != 0)
        memset(&s_calibrationData, 0, sizeof(s_calibrationData));
}

/* ============================================================
 * WAKE CALIBRATION SYSTEM
 * ============================================================ */

void begin_wake_calibration(void) {
    memset(s_wakeSumSpeed, 0, sizeof(s_wakeSumSpeed));
    memset(s_wakeCntBin,   0, sizeof(s_wakeCntBin));
    s_isWakeCalibrating = true;
}

bool end_wake_calibration(void) {
    uint8_t valid_bins = 0;
    float bin_avg[WAKE_BINS];
    float global_sum = 0.0f;

    for (int b = 0; b < WAKE_BINS; b++) {
        if (s_wakeCntBin[b] >= WAKE_CAL_MIN_PER_BIN) {
            bin_avg[b] = (float)s_wakeSumSpeed[b] / (float)s_wakeCntBin[b];
            global_sum += bin_avg[b];
            valid_bins++;
        } else {
            bin_avg[b] = -1.0f; 
        }
    }

    if (valid_bins < WAKE_CAL_MIN_COVERAGE) {
        s_isWakeCalibrating = false;
        return false;
    }

    float global_mean = global_sum / (float)valid_bins;

    const int strut_bins[4] = { 0, 9, 18, 27 }; 
    float A_sum     = 0.0f;
    float sigma_sum = 0.0f;
    uint8_t fit_ok  = 0;

    for (int s = 0; s < 4; s++) {
        float min_speed = 1e9f;
        int   min_bin   = -1;
        for (int db = -2; db <= 2; db++) {
            int b = (strut_bins[s] + db + WAKE_BINS) % WAKE_BINS;
            if (bin_avg[b] > 0 && bin_avg[b] < min_speed) {
                min_speed = bin_avg[b];
                min_bin   = b;
            }
        }
        if (min_bin < 0) continue;

        float A = (global_mean - min_speed) / global_mean;
        if (A < 0.02f || A > 0.25f) continue;

        float half_att_speed = min_speed + (global_mean - min_speed) * 0.5f;
        float sigma_est = 0.0f;

        for (int db = 1; db <= 5; db++) {
            int b_pos = (min_bin + db) % WAKE_BINS;
            int b_neg = (min_bin - db + WAKE_BINS) % WAKE_BINS;
            bool pos_ok = (bin_avg[b_pos] >= 0 && bin_avg[b_pos] >= half_att_speed);
            bool neg_ok = (bin_avg[b_neg] >= 0 && bin_avg[b_neg] >= half_att_speed);
            if (pos_ok || neg_ok) {
                sigma_est = (float)(db * 10) / 2.355f;
                break;
            }
        }

        if (sigma_est < 5.0f || sigma_est > 35.0f) {
            sigma_est = WAKE_DEFAULT_SIGMA_DEG; 
        }

        A_sum     += A;
        sigma_sum += sigma_est;
        fit_ok++;
    }

    if (fit_ok == 0) {
        s_isWakeCalibrating = false;
        return false;
    }

    float A_fit     = A_sum     / (float)fit_ok;
    float sigma_fit = sigma_sum / (float)fit_ok;

    if (A_fit     < 0.02f) A_fit     = 0.02f;
    if (A_fit     > 0.25f) A_fit     = 0.25f;
    if (sigma_fit < 5.0f)  sigma_fit = 5.0f;
    if (sigma_fit > 35.0f) sigma_fit = 35.0f;

    s_wakeCalib.A_q15       = (int16_t)(A_fit     * 32768.0f);
    s_wakeCalib.sigma_deg10 = (int16_t)(sigma_fit * 10.0f);
    storeWakeCalibration();

    s_isWakeCalibrating = false;
    return true;
}

bool is_wake_calibrating(void) {
    return s_isWakeCalibrating;
}

uint16_t get_wake_cal_progress(uint8_t bin) {
    if (bin >= WAKE_BINS) return 0;
    return s_wakeCntBin[bin];
}

void storeWakeCalibration(void) {
    s_wakeCalib.crc = crc8_dallas(&s_wakeCalib, sizeof(s_wakeCalib) - 1, 0xEE);
    WriteEEPROM(WAKE_CALIB_ADDRESS, &s_wakeCalib, sizeof(s_wakeCalib));
}

void recallWakeCalibration(void) {
    readEEPROM(WAKE_CALIB_ADDRESS, &s_wakeCalib, sizeof(s_wakeCalib));
    if (crc8_dallas(&s_wakeCalib, sizeof(s_wakeCalib), 0xEE) != 0) {
        s_wakeCalib.A_q15       = WAKE_A_Q15_DEFAULT;      
        s_wakeCalib.sigma_deg10 = WAKE_SIGMA_DEG10_DEFAULT; 
    }
}

/* ============================================================
 * EEPROM READ / WRITE
 * ============================================================ */

void WriteEEPROM(uint16_t biasAddress, void *data, uint16_t len) {
    HAL_FLASHEx_DATAEEPROM_Unlock();
    uint8_t *p    = (uint8_t *)data;
    uint32_t addr = FLASH_EEPROM_BASE + biasAddress;
    for (uint16_t i = 0; i < len; i++)
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, addr + i, p[i]);
    HAL_FLASHEx_DATAEEPROM_Lock();
}

void readEEPROM(uint16_t biasAddress, void *data, uint16_t len) {
    memcpy(data, (void *)(FLASH_EEPROM_BASE + biasAddress), len);
}
