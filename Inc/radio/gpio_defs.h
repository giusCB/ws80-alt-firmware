#pragma once
#include "stm32l1xx_hal_gpio.h"

#define CMT_GPIO1_GPIO GPIOB
//#define CMT_GPIO2_GPIO GPIOC
//#define CMT_GPIO3_GPIO GPIOC
#define CMT_GPIO1_GPIO_PIN (1 << 12)
//#define CMT_GPIO2_GPIO_PIN
//#define CMT_GPIO3_GPIO_PIN

#define CMT_CSB_GPIO GPIOC
#define CMT_FCSB_GPIO GPIOC
#define CMT_SCLK_GPIO GPIOC
#define CMT_SDIO_GPIO GPIOC


#define CMT_CSB_GPIO_PIN  (1 << 7)
#define CMT_FCSB_GPIO_PIN (1 << 6)
#define CMT_SCLK_GPIO_PIN (1 << 9)
#define CMT_SDIO_GPIO_PIN (1 << 8)

void GPIO_Pin_Setting(GPIO_TypeDef *gpio, 
    uint16_t nPin, uint32_t speed, uint32_t mode, uint32_t pull);

#define SET_GPIO_OUT(x)             GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL)
#define SET_GPIO_IN(x)              GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_INPUT, GPIO_NOPULL)
#define SET_GPIO_IN_PU(x)           GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_INPUT, GPIO_PULLUP)
#define SET_GPIO_OD(x)              GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL)
#define SET_GPIO_AIN(x)             GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_ANALOG, GPIO_NOPULL)
#define SET_GPIO_AFOUT(x)           GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_AF_PP, GPIO_NOPULL)
#define SET_GPIO_AFOD(x)            GPIO_Pin_Setting(x, x##_PIN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_MODE_AF_OD, GPIO_NOPULL)
#define SET_GPIO_H(x)               (x->BSRR = x##_PIN) //GPIO_SetBits(x, x##_PIN)
#define SET_GPIO_L(x)               (x->BSRR  = x##_PIN << 16) //GPIO_ResetBits(x, x##_PIN)
#define READ_GPIO_PIN(x)            (((x->IDR & x##_PIN)!=Bit_RESET) ?1 :0) //GPIO_ReadInputDataBit(x, x##_PIN) 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;