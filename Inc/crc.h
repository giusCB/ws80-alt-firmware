#include <stdint.h>

uint8_t crc8_dallas(void* data, uint8_t len, uint8_t initial);
uint8_t checksum(void* data, uint8_t len);