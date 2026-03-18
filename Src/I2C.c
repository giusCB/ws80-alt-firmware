// Bitbanged I2C, copied from the decompiled firmware.
//#define DEBUG_I2C
#include <stdbool.h>
#include "stm32l1xx.h"
#include "I2C.h"
#ifdef HAL_I2C
#include "stm32l1xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c1;
#endif

const uint8_t SDA = 7; // SDA - data line
const uint8_t SCL = 6; // SCL - clock line
GPIO_TypeDef* const I2C_GPIO = GPIOB;

bool I2CSendByte(uint8_t byte);
void I2CStart();
void I2CStop();

void Delay_micros(uint32_t amt)
{
    uint32_t start = SysTick->VAL;
    uint32_t ticks = amt * 32;
    while (SysTick->VAL - start < ticks)
    {}
}

static void I2CDelay() { Delay_micros(10); }

void InitI2C()
{
    // Set to open drain
    I2C_GPIO->OTYPER |= (1 << SDA) | (1 << SCL);
    // Set to output
    I2C_GPIO->MODER = (I2C_GPIO->MODER & ~((GPIO_MODER_MODER0_Msk << (SDA * 2)) | (GPIO_MODER_MODER0_Msk << (SCL * 2)))) 
                        | (1 << (SDA * 2) | 1 << (SCL * 2));
    I2C_GPIO->PUPDR = (I2C_GPIO->PUPDR & ~((GPIO_PUPDR_PUPDR0_Msk << (SDA * 2)) | (GPIO_PUPDR_PUPDR0_Msk << (SCL * 2))))
                        | (1 << (SDA * 2) | 1 << (SCL * 2));
    // Set HIGH Z
    I2C_GPIO->BSRR = (1 << SDA) | (1 << SCL);
    I2CDelay();
}

void I2CStart()
{
    I2C_GPIO->BSRR = 0x10000 << SDA;
    I2CDelay();
    I2C_GPIO->BSRR = 0x10000 << SCL;
    I2CDelay();
}

void I2CStop()
{
    I2C_GPIO->BSRR = 0x1 << SCL;
    I2CDelay();
    I2C_GPIO->BSRR = 0x1 << SDA;
    I2CDelay();
}

void I2CSendBit(uint8_t bit)
{
    I2C_GPIO->BSRR = 1 << (SDA + 16 * (1 - bit));
    I2CDelay();
    I2C_GPIO->BSRR = 1 << SCL;
    I2CDelay();
    I2C_GPIO->BSRR = 0x10000 << SCL;
    I2CDelay();
}

bool I2CReadBit()
{
    // Set SDA High-Z:
    I2C_GPIO->BSRR = 1 << SDA;
    I2CDelay();
    // Set clock high
    I2C_GPIO->BSRR = 1 << SCL;
    I2CDelay();
    // Read data line
    uint8_t bit = (I2C_GPIO->IDR >> SDA) & 1;
    // Pull clock low
    I2C_GPIO->BSRR = 0x10000 << SCL;
    I2CDelay();
    return bit;
}

bool I2CSendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        uint8_t thisBit = ((byte >> 7) & 1);
        I2CSendBit(thisBit);
        byte <<= 1;
    }
    return !I2CReadBit();
}

uint8_t I2CReadByte()
{
    I2C_GPIO->BSRR = 1 << SDA;
    I2CDelay();
    I2C_GPIO->BSRR = 0x10000 << SDA;
    I2CDelay();

    uint8_t ret = 0;
    for (int i = 0; i < 8; i++)
    {
        ret <<=1;
        ret |= I2CReadBit();
    }
    return ret;
}

bool I2CReadFromAddress(uint8_t address, uint8_t* buffer, uint8_t len)
{
    #ifdef HAL_I2C
    return HAL_I2C_Master_Receive(&hi2c1, lightAddress, &ret, 1, 20) == HAL_OK;
    #else
    I2CStart();
    bool success = I2CSendByte(address | 1);
    if (success)
        for (int i = 0; i < len; i++)
        {
            buffer[i] = I2CReadByte();
            if (i != len - 1)
                I2CSendBit(0);
            else
                I2CSendBit(1);
        }
    I2CStop();
    return success;
    #endif
}

bool I2CWriteToAddress(uint8_t address, uint8_t* buffer, uint8_t len)
{
    #ifdef HAL_I2C
    return HAL_I2C_Master_Transmit(&hi2c1, address, buffer, len, 20) == HAL_OK;
    #else
    I2CStart();
    bool success = I2CSendByte(address | 0);
    if (success)
        for (int i = 0; i < len; i++)
        {
            if (!I2CSendByte(buffer[i]))
            {
                success = false;
                break;
            }
        }
    I2CStop();
    return success;
    #endif
}


void WaitForStop()
{
    #ifdef HAL_I2C
    uint32_t entryTicks = HAL_GetTick();
    while (I2C1->CR1 & I2C_CR1_STOP)
    { 
        if (HAL_GetTick() - entryTicks > 5)
        {
            LIGHT_PRINT("TIMEOUT waiting for stop!");
            break;
        }
    }
    #endif
}

void PrintRegisters()
{
    for (int i = 0; i < 2; i++)
    {
        uint32_t cr1 = I2C1->CR1;
        uint32_t cr2 = I2C1->CR2;
        uint32_t sr1 = I2C1->SR1;
        uint32_t sr2 = I2C2->SR2;
        debug_print("After read val. CR1: %08lx, CR2: %08lx, SR1: %08lx, SR2: %08lx\r\n",
            cr1, cr2, sr1, sr2);
        HAL_Delay(1);
    }
}