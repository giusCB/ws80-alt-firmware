/**
 * @file wind_calc.c
 * @brief Professional Ultrasonic Wind Calculation Module - Industrial Grade
 * Final Seal: Deterministic normalization, UB-free shifts, and geometric floor.
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
#define MEDIAN_WINDOW 7
#define MAX_ACCEL_CMPS 2500  
#define qn_13_pi 25736
#define qn_14_one 16384
#define qn_14_sqrt1_2 11585

/* --- 3. STATIC VARIABLES & STATE --- */
static int16_t rawBufferX[MEDIAN_WINDOW];
static int16_t rawBufferY[MEDIAN_WINDOW];
static int16_t lastFilteredX = 0, lastFilteredY = 0;
static uint8_t medianIdx = 0;
static bool bufferFull = false;
static bool firstSampleReceived = false;

static bool s_isCalibrating = false; 
static uint16_t s_calibrationCount = 0; 

const uint8_t VectorSumSize = 4;
int16_t lastSampleX = 0, lastSampleY = 0;
static int32_t spdVectorSumX, spdVectorSumY;
static int32_t dirVectorSumX, dirVectorSumY;
static uint32_t spdSum = 0;
static uint16_t maxSpd = 0;
uint16_t spdVectorSumCnt = 0, dirVectorSumCnt = 0, spdSumCnt = 0;

const uint16_t calibrationAddress = 0xa4;
q2_13 s_signalPhases[6][2];

struct __attribute__((packed, aligned(2))) calibrationStruct {
    q2_13 phases[6];
    uint8_t crc;
};
struct calibrationStruct s_calibrationData;
q18_13 s_calibrationPhaseSums[6];

/* --- 4. PROTOTYPES --- */
uint32_t FastIntSqrt(uint32_t value);
void storeCalibration(void);
void WriteEEPROM(uint16_t biasAddress, void* data, uint16_t len);
void readEEPROM(uint16_t biasAddress, void* data, uint16_t len);
q18_13 normalise_angle(q18_13 angle);
void get_radius_vector_cmps(int16_t* x_cmps, int16_t* y_cmps, q18_13 phaseDiff, uint8_t channel);
int16_t get_speed_cmps(uint8_t channel, int32_t timeDiff_ns);

/* --- 5. SAFE MATH HELPERS --- */

static q15_t safe_scale_to_q15(int32_t v, int8_t shift) {
    if (v == 0) return 0;
    int64_t mag = (v < 0) ? -(int64_t)v : (int64_t)v;
    uint64_t scaled = (shift >= 0) ? ((uint64_t)mag << shift) : ((uint64_t)mag >> (-shift));
    if (scaled > 32767ULL) scaled = 32767ULL;
    return (v < 0) ? -(q15_t)scaled : (q15_t)scaled;
}

q18_13 normalise_angle(q18_13 angle) {
    int32_t period = 2 * qn_13_pi;
    // Deterministic normalization using modulo to prevent CPU lockup on large offsets
    angle = ((angle + qn_13_pi) % period + period) % period - qn_13_pi;
    return angle;
}

/* --- 6. MATH TABLES & VECTORS --- */
static int16_t sin24_5[49] = {0,519,1005,1425,1751,1963,2047,1997,1816,1516,1117,645,131,-391,-889,-1328,-1680,-1922,-2039,-2022,-1873,-1601,-1225,-769,-262,262,769,1225,1601,1873,2022,2039,1922,1680,1328,889,391,-131,-645,-1117,-1516,-1816,-1997,-2047,-1963,-1751,-1425,-1005,-519};
static int16_t cos24_5[49] = {2048,1981,1784,1471,1062,583,66,-456,-947,-1377,-1716,-1944,-2044,-2010,-1845,-1559,-1172,-707,-197,327,829,1277,1641,1898,2031,2031,1898,1641,1277,829,327,-197,-707,-1172,-1559,-1845,-2010,-2044,-1944,-1716,-1377,-947,-456,66,583,1062,1471,1784,1981};
static int16_t window[245] = {1,3,6,11,17,24,33,43,54,66,80,95,112,130,148,169,190,213,236,261,288,315,343,373,404,435,468,502,537,572,609,647,685,725,765,806,848,891,935,979,1024,1070,1116,1163,1210,1258,1307,1356,1405,1455,1505,1556,1607,1658,1710,1761,1813,1865,1917,1970,2022,2074,2126,2179,2231,2283,2335,2283,2231,2179,2126,2074,2022,1970,1917,1865,1813,1761,1710,1658,1607,1556,1505,1455,1405,1356,1307,1258,1210,1163,1116,1070,1024,979,935,891,848,806,765,725,685,647,609,572,537,502,468,435,404,373,343,315,288,261,236,213,190,169,148,130,112,95,80,66,54,43,33,24,17,11,6,3,1};

q1_14 s_radiusVectors[6][2] = { { 11585, -11585 }, { -11585, -11585 }, { 0, -16384 }, { -16384, 0 }, { -11585, -11585 }, { 11585, -11585 } };
q1_14 s_tangentVectors[6][2] = { { 11585, 11585 }, { 11585, -11585 }, { 16384, 0 }, { 0, 16384 }, { 11585, -11585 }, { 11585, 11585 } };

/* --- 7. CORE LOGIC --- */

static int16_t get_median_7(int16_t *data) {
    int16_t temp[MEDIAN_WINDOW];
    memcpy(temp, data, MEDIAN_WINDOW * sizeof(int16_t));
    for (uint8_t i = 1; i < MEDIAN_WINDOW; i++) {
        int16_t key = temp[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    return temp[3];
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
    uint32_t maxVal = (uAbsCos > uAbsSin) ? uAbsCos : uAbsSin;

    q15_t finalCos = 0, finalSin = 0;
    if (maxVal > 0) {
        int8_t shift = (int8_t)__builtin_clz(maxVal) - 17;
        finalCos = safe_scale_to_q15(cosSum, shift);
        finalSin = safe_scale_to_q15(sinSum, shift);
    }

    int32_t c32 = (int32_t)finalCos;
    int32_t s32 = (int32_t)finalSin;
    g_signalPowers[channel][direction] = (uint32_t)(c32 * c32 + s32 * s32);
    arm_atan2_q15(finalSin, finalCos, &s_signalPhases[channel][direction]);
}

void calculate_wind(int16_t *x_cmps, int16_t *y_cmps) {
    q18_13 signalPhaseDiffs[6];
    for (uint8_t ch = 0; ch < 6; ch++) {
        signalPhaseDiffs[ch] = normalise_angle((q18_13)s_signalPhases[ch][0] - s_signalPhases[ch][1] - s_calibrationData.phases[ch]);
    }
    uint32_t best_res = 0xFFFFFFFF;
    int16_t best_x = 0, best_y = 0;

    for (int8_t ch2W = -1; ch2W <= 1; ch2W++) {
        for (int8_t ch3W = -1; ch3W <= 1; ch3W++) {
            int32_t sum_x = 0, sum_y = 0;
            int16_t rV[6][2];
            for (uint8_t ch = 0; ch < 6; ch++) {
                q18_13 pD = signalPhaseDiffs[ch];
                if (ch == 2) pD += 2 * qn_13_pi * ch2W;
                if (ch == 3) pD += 2 * qn_13_pi * ch3W;
                get_radius_vector_cmps(&rV[ch][0], &rV[ch][1], pD, ch);
                sum_x += (int32_t)rV[ch][0]; sum_y += (int32_t)rV[ch][1];
            }
            int16_t bX = (int16_t)(sum_x / 3), bY = (int16_t)(sum_y / 3);
            uint32_t res = 0;
            for (uint8_t ch = 0; ch < 6; ch++) {
                int32_t dx = (int32_t)bX - rV[ch][0]; 
                int32_t dy = (int32_t)bY - rV[ch][1];
                int64_t dot_t = ((int64_t)dx * s_tangentVectors[ch][0] + (int64_t)dy * s_tangentVectors[ch][1]) >> 14;
                int64_t diff = (int64_t)dx * dx + (int64_t)dy * dy - (dot_t * dot_t);
                if (diff < 0) diff = 0; 
                res += (uint32_t)(diff >> 2);
            }
            if (res < best_res) { best_res = res; best_x = bX; best_y = bY; }
        }
    }
    *x_cmps = best_x; *y_cmps = best_y;
}

void store_wind_sample(int16_t x_cmps, int16_t y_cmps) {
    if (!firstSampleReceived) {
        for (int i = 0; i < MEDIAN_WINDOW; i++) { rawBufferX[i] = x_cmps; rawBufferY[i] = y_cmps; }
        lastFilteredX = x_cmps; lastFilteredY = y_cmps;
        firstSampleReceived = true; bufferFull = true;
    }
    rawBufferX[medianIdx] = x_cmps; rawBufferY[medianIdx] = y_cmps;
    medianIdx = (medianIdx + 1) % MEDIAN_WINDOW;

    int16_t medX = get_median_7(rawBufferX);
    int16_t medY = get_median_7(rawBufferY);

    if (bufferFull) {
        int16_t dX = medX - lastFilteredX, dY = medY - lastFilteredY;
        if (abs(dX) > MAX_ACCEL_CMPS) medX = lastFilteredX + (dX > 0 ? MAX_ACCEL_CMPS : -MAX_ACCEL_CMPS);
        if (abs(dY) > MAX_ACCEL_CMPS) medY = lastFilteredY + (dY > 0 ? MAX_ACCEL_CMPS : -MAX_ACCEL_CMPS);
    }
    lastFilteredX = medX; lastFilteredY = medY;
    lastSampleX = medX; lastSampleY = medY;
    
    dirVectorSumX += (int32_t)medX; dirVectorSumY += (int32_t)medY; dirVectorSumCnt++;
    spdVectorSumX += (int32_t)medX; spdVectorSumY += (int32_t)medY; spdVectorSumCnt++;

    if (spdVectorSumCnt >= VectorSumSize) {
        int32_t avgX = spdVectorSumX / (int32_t)spdVectorSumCnt;
        int32_t avgY = spdVectorSumY / (int32_t)spdVectorSumCnt;
        uint16_t spd = (uint16_t)FastIntSqrt((uint32_t)((int64_t)avgX * avgX + (int64_t)avgY * avgY));
        spdSum += (uint32_t)spd; if (spd > maxSpd) maxSpd = spd;
        spdSumCnt++; spdVectorSumX = 0; spdVectorSumY = 0; spdVectorSumCnt = 0;
    }
}

void get_wind_parameters(uint16_t* pAvg_dmps, uint16_t* pGust_dmps, uint16_t* pAngle_deg) {
    if (spdSumCnt > 0) *pAvg_dmps = (uint16_t)((spdSum + (spdSumCnt * 5)) / (spdSumCnt * 10));
    else *pAvg_dmps = 0;
    *pGust_dmps = (uint16_t)(maxSpd / 10);
    
    if (dirVectorSumCnt > 0 && (dirVectorSumX != 0 || dirVectorSumY != 0)) {
        int32_t avgX = dirVectorSumX / (int32_t)dirVectorSumCnt;
        int32_t avgY = dirVectorSumY / (int32_t)dirVectorSumCnt;
        uint32_t maxV = (uint32_t)(abs(avgX) > abs(avgY) ? abs(avgX) : abs(avgY));
        int8_t shift = (int8_t)__builtin_clz(maxV) - 17;
        q15_t fY = safe_scale_to_q15(avgY, shift);
        q15_t fX = safe_scale_to_q15(avgX, shift);
        q15_t angleQ;
        arm_atan2_q15(fY, fX, &angleQ);
        int32_t deg = (int32_t)(angleQ * 3667 + (1 << 18)) >> 19;
        while (deg < 0) deg += 360;
        *pAngle_deg = (uint16_t)(deg % 360);
    } else *pAngle_deg = 0;
    spdSum = 0; spdSumCnt = 0; dirVectorSumCnt = 0; dirVectorSumX = 0; dirVectorSumY = 0; maxSpd = 0;
}

int16_t get_speed_cmps(uint8_t channel, int32_t timeDiff_ns) {
    int32_t v_s = 3315 + (g_tempMeasurement * 61) / 100;
    int32_t L = (channel == 2 || channel == 3) ? 35700 : 16000;
    return (int16_t)(((v_s * v_s) / L) * timeDiff_ns / 2000);
}

void get_radius_vector_cmps(int16_t* x_cmps, int16_t* y_cmps, q18_13 phaseDiff, uint8_t channel) {
    int32_t delay = (phaseDiff * 3900) >> 13;
    int16_t spd = get_speed_cmps(channel, delay);
    *x_cmps = (int16_t)((int32_t)spd * s_radiusVectors[channel][0] >> 14);
    *y_cmps = (int16_t)((int32_t)spd * s_radiusVectors[channel][1] >> 14);
}

uint32_t FastIntSqrt(uint32_t value) {
    if (!value) return 0;
    uint32_t xn = 1 << ((32 - __CLZ (value))/2);
    xn = (xn + value/xn)/2; xn = (xn + value/xn)/2; xn = (xn + value/xn)/2;
    return xn;
}

/* --- 8. CALIBRATION & EEPROM --- */

void begin_calibration(void) {
    memset(s_calibrationPhaseSums, 0, sizeof(s_calibrationPhaseSums));
    s_calibrationCount = 0;
    s_isCalibrating = true;
}

void store_calibration_sample(void) {
    if (!s_isCalibrating) return;
    for (int i = 0; i < 6; i++) s_calibrationPhaseSums[i] += normalise_angle(s_signalPhases[i][0] - s_signalPhases[i][1]);
    s_calibrationCount++;
}

bool maybe_end_calibration(void) {
    if (!s_isCalibrating || s_calibrationCount < 200) return false;
    for (int i = 0; i < 6; i++) s_calibrationData.phases[i] = (int16_t)(s_calibrationPhaseSums[i] / (int32_t)s_calibrationCount);
    storeCalibration();
    s_isCalibrating = false; 
    s_calibrationCount = 0;
    return true;
}

void storeCalibration(void) {
    s_calibrationData.crc = crc8_dallas(&s_calibrationData, sizeof(s_calibrationData) - 1, 0xEE);
    WriteEEPROM(calibrationAddress, &s_calibrationData, sizeof(s_calibrationData));
}

void recallCalibration(void) {
    readEEPROM(calibrationAddress, &s_calibrationData, sizeof(s_calibrationData));
    if (crc8_dallas(&s_calibrationData, sizeof(s_calibrationData), 0xEE) != 0) memset(&s_calibrationData, 0, sizeof(s_calibrationData));
}

void WriteEEPROM(uint16_t biasAddress, void* data, uint16_t len) {
    HAL_FLASHEx_DATAEEPROM_Unlock();
    uint8_t* p = (uint8_t*)data;
    uint32_t addr = FLASH_EEPROM_BASE + biasAddress;
    for (uint16_t i = 0; i < len; i++) HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, addr + i, p[i]);
    HAL_FLASHEx_DATAEEPROM_Lock();
}

void readEEPROM(uint16_t biasAddress, void* data, uint16_t len) {
    memcpy(data, (void*)(FLASH_EEPROM_BASE + biasAddress), len);
}
