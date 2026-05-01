/* USER CODE BEGIN Header */
/**
 * Professional Weather Station Firmware - Core Loop
 * Fixed: All unused function warnings resolved by proper initialization.
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "wind.h"
#include "temperature.h"
#include "my_time.h"
#include "radio.h"
#include "light.h"
#include "battery.h"
#include <stdbool.h>

/* USER CODE BEGIN 0 */
extern volatile uint32_t g_rtcTicks;
/* USER CODE END 0 */

/* --- HSI Calibration Constants --- */
#define LSE_FREQ_HZ                 32768U
#define HSI_TARGET_FREQ             16000000U
#define HSI_TO_TIMCLK_RATIO         2U 
#define EXPECTED_TICKS_ACCUMULATED  ((HSI_TARGET_FREQ * HSI_TO_TIMCLK_RATIO * 1024U) / LSE_FREQ_HZ) 
#define TARGET_CAPTURES             128U

#define HSITRIM_POS      __builtin_ctz(RCC_ICSCR_HSITRIM)
#define HSITRIM_MAX      31U
#define HSITRIM_MIN      0U
#define HSITRIM_DEADBAND 20U 

#define TIM5_OR_TI4RMP_Pos  6U
#define TIM5_OR_TI4RMP_Msk  (0x3U << TIM5_OR_TI4RMP_Pos)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

#ifdef HAL_I2C
I2C_HandleTypeDef hi2c1;
#endif

IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef  hrtc;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim9;
TIM_HandleTypeDef  htim5; 

volatile float g_hsi_actual_freq = 16000000.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
#ifdef HAL_I2C
static void MX_I2C1_Init(void);
#endif
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void SetUnusedAnalog(void);

void check_calibration_A2(void);
void Start_HSI_Measurement(TIM_HandleTypeDef *htim);
bool Process_HSI_Calibration(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN 1 */
void InitScope(void);
/* USER CODE END 1 */

/* ---------------------------------------------------------------------------*/
int main(void)
{
    /* Reset di sistema */
    GPIOA->ODR = 1;
    RCC->CFGR &= ~RCC_CFGR_SW_Msk;

    HAL_Init();
    SystemClock_Config();

    /* Inizializzazione Periferiche - Ordine corretto per evitare warning */
    MX_GPIO_Init();
    MX_DMA_Init();   // Risolve warning DMA
    MX_ADC_Init();   // Risolve warning ADC
#ifdef HAL_I2C
    MX_I2C1_Init();
#endif
    MX_RTC_Init();
    MX_TIM5_Init();
    MX_TIM2_Init();  // Risolve warning Timer 2
    MX_TIM9_Init();  // Risolve warning Timer 9
    MX_IWDG_Init();  // Risolve warning Watchdog (Cane da guardia)

#ifdef DEBUG
    MX_USB_DEVICE_Init();
#endif

    InitScope();
    init_radio();

    static uint32_t lastCalTick  = 0;
    bool            is_measuring_hsi = false;

    while (1)
    {
        /* Nutre il cane da guardia per evitare reset indesiderati */
        HAL_IWDG_Refresh(&hiwdg);

        // 1. Lettura sensori
        processRadio();
        processLight();
        processBattery();
        process_wind();
        ProcessTemperature();
        check_calibration_A2();

        // 2. Calibrazione HSI periodica
        if ((g_rtcTicks - lastCalTick) >= 512U)
        {
            lastCalTick = g_rtcTicks;
            if (!is_measuring_hsi)
            {
                Start_HSI_Measurement(&htim5);
                is_measuring_hsi = true;
            }
        }

        if (is_measuring_hsi)
        {
            if (Process_HSI_Calibration(&htim5))
            {
                is_measuring_hsi = false;
            }
        }

        // 3. Gestione Risparmio Energetico
        if (doContinuousMonitoring())
        {
            // Resta sveglio
        }
        else
        {
            if (is_measuring_hsi)
            {
                HAL_TIM_IC_Stop(&htim5, TIM_CHANNEL_4);
                __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC4);
                is_measuring_hsi = false;
            }
            stopLowPower();
        }
    }
}

/* --- Implementazione Funzioni di Calibrazione --- */

void Start_HSI_Measurement(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_IC_Start(htim, TIM_CHANNEL_4);
}

bool Process_HSI_Calibration(TIM_HandleTypeDef *htim)
{
    static uint32_t first_capture = 0;
    static uint16_t capture_count = 0;
    static bool     is_first      = true;

    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) == RESET)
        return false;

    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);

    if (is_first)
    {
        first_capture = cap;
        capture_count = 0;
        is_first      = false;
        return false;
    }

    if (++capture_count < TARGET_CAPTURES)
        return false;

    HAL_TIM_IC_Stop(htim, TIM_CHANNEL_4);
    is_first = true; 

    uint32_t delta = cap - first_capture;
    g_hsi_actual_freq = ((float)delta * (float)LSE_FREQ_HZ) / (1024.0f * (float)HSI_TO_TIMCLK_RATIO);

    uint32_t trim = (RCC->ICSCR & RCC_ICSCR_HSITRIM) >> HSITRIM_POS;
    if      (delta > EXPECTED_TICKS_ACCUMULATED + HSITRIM_DEADBAND && trim > HSITRIM_MIN) trim--;
    else if (delta < EXPECTED_TICKS_ACCUMULATED - HSITRIM_DEADBAND && trim < HSITRIM_MAX) trim++;

    uint32_t icscr = RCC->ICSCR;
    icscr &= ~RCC_ICSCR_HSITRIM;
    icscr |= (trim << HSITRIM_POS);
    RCC->ICSCR = icscr;

    return true;
}

void check_calibration_A2(void)
{
    static uint16_t lowEntryMillis = 0;
    static uint32_t lowEntryTicks  = 0;
    static bool     lastState      = true;

    bool thisState = (GPIOA->IDR & GPIO_PIN_2) != 0;

    if (thisState == false && lastState == true)
    {
        lowEntryMillis = millis1024();
        lowEntryTicks  = g_rtcTicks;
    }
    else if (thisState == true && lastState == false)
    {
        if (g_rtcTicks - lowEntryTicks > 2 ||
            (lowEntryMillis + 1024 - millis1024()) % 1024 > 102)
            calibrateWind();
    }
    lastState = thisState;
}

/* --- Configurazioni Hardware --- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
#ifdef USE_HSE
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
#else
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
#endif
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    FLASH->ACR |= FLASH_ACR_PRFTEN;

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
    hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
    hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc.Init.DMAContinuousRequests = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) Error_Handler();
}

#ifdef HAL_I2C
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
#endif

static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

static void MX_RTC_Init(void)
{
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 31;
    hrtc.Init.SynchPrediv = 1024;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();

    const uint32_t wakeUpCounter = RTC_FREQ / WAKEUP_FREQUENCY / 16;
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
        Error_Handler();
}

static void MX_TIM5_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM_IC_InitTypeDef sConfigIC = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFFU;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim5);

    htim5.Instance->OR &= ~TIM5_OR_TI4RMP_Msk;
    htim5.Instance->OR |=  TIM5_OR_TI4RMP_Msk; 

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);
}

static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 6144;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
}

static void MX_TIM9_Init(void)
{
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 0;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 400;
    if (HAL_TIM_Base_Init(&htim9) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void SetUnusedAnalog(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    SetUnusedAnalog();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        GPIOA->ODR ^= 1;
        HAL_Delay(100);
    }
}
