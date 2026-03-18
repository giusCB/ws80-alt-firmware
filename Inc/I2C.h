#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "debug.h"

void InitI2C();
bool I2CReadFromAddress(uint8_t address, uint8_t* buffer, uint8_t len);
bool I2CWriteToAddress(uint8_t address, uint8_t* buffer, uint8_t len);
void Delay_micros(uint32_t amt);
void WaitForStop();
