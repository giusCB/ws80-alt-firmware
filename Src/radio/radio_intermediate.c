#include "radio.h"
#include "wind.h"
#include "temperature.h"
#include "my_time.h"
#include "crc.h"
#include <stdbool.h>
#include <string.h>

#define ID_ADDRESS 0x80

const uint8_t radio_tx_interval = (uint8_t)(RADIO_TX_PERIOD * WAKEUP_FREQUENCY);

uint32_t last_tx = 0;
uint32_t radio_id;
//TODO: All of these
int16_t light = 0; 
uint8_t battery = 0;
uint8_t pressure = 0;
uint8_t uvIndex_x10 = 0;


struct radioPacket
{
    uint8_t b1;
    uint8_t id[3];
    uint8_t light_hi;
    uint8_t light_lo;
    uint8_t battery;
    uint8_t hiBits;
    uint8_t temp_lo;
    uint8_t humidity;
    uint8_t wind_avg_lo;
    uint8_t wind_dir_lo;
    uint8_t wind_max_lo;
    uint8_t uv_index_x10;
    uint8_t pressureAndStatus;
    uint8_t pressure;
    uint8_t crc1;
    uint8_t chkSum;
} typedef RadioPacketTypedef;

void read_EEPROM_skipping(uint16_t src_addr, void *dest, int16_t count);
void LoadRadioId();
void LoadFrequencySelector();
void LoadRadioIdFromDeviceID();
bool LoadRadioIdFromEEPROM();
void CreateRadioPacket(RadioPacketTypedef *packet);


void init_radio()
{
    RADIO_PRINT("Radio init, press C...\r\n");
    wait_for_continue();
    LoadRadioId();
    LoadFrequencySelector();
    configure_radio(true, g_frequencySelector);
    test_radio();
    configure_radio(false, g_frequencySelector);
}

void LoadFrequencySelector()
{
    GPIOC->PUPDR = (GPIOC->PUPDR 
        & ~((3 << 10 * 2) | (3 << 11 * 2)))
        | (1 << 10 * 2) | (1 << 11 * 2);
    // Not sure how long it takes for the pull up resistor to work.
    // We'll wait 1/2 us.
    uint32_t entryTicks = SysTick->VAL; // 1 cycle
    uint32_t st_load = SysTick->LOAD; // 1 cycle
    entryTicks += st_load; // 1 cycle
    uint32_t diff;
    do
    {
        diff = (entryTicks - SysTick->VAL) % st_load; // 3 cycles
    } while (diff < 16); // 2 cycles

    switch(GPIOC->IDR >> 10 & 3) {
    case 0:
        g_frequencySelector = 3;
      break;
    case 1:
        g_frequencySelector = 0;
      break;
    case 2:
        g_frequencySelector = 1;
        break;
    case 3:
        g_frequencySelector = 2;
    }
    GPIOC->PUPDR = (GPIOC->PUPDR 
        & ~((3 << 10 * 2) | (3 << 11 * 2)));
    RADIO_PRINT("Radio frequency selector: %d\r\n", g_frequencySelector);
}

void LoadRadioId()
{
    if (!LoadRadioIdFromEEPROM())
    {
        RADIO_PRINT("Loading ID from DeviceID: ");
        LoadRadioIdFromDeviceID();
    }
    else
        RADIO_PRINT("Loaded ID from EEPROM: ");
    RADIO_PRINT("%08lx\r\n", radio_id);
}

void LoadRadioIdFromDeviceID()
{
    uint32_t* DEVICE_ID_2 = (uint32_t*)(UID_BASE + 0x14);
    radio_id = *DEVICE_ID_2;
    if (radio_id == 0)
    {
        const void *const FACTORY_INFO_START = (void*) 0x1FF80020;
        const void *const infoEnd = (void*) 0x1ff8007e;
        for (const uint32_t* p = FACTORY_INFO_START; p < (uint32_t*)infoEnd; p += 2)
        {
            uint32_t val = *p ^ *(p + 1);
            if (val != 0)
            {
                radio_id = val;
                return;
            }
        }
    }
}

bool LoadRadioIdFromEEPROM()
{
    uint16_t id[2];
    read_EEPROM_skipping(ID_ADDRESS, &id, 2);
    RADIO_PRINT("EEPROM ID Data: %04x %04x\r\n", 
        id[0], id[1]);
    if (((uint8_t*)id)[1] != 'Z')
        return false;
    radio_id = ((uint32_t)id[0] & 0xFF)  << 16 | id[1];
    return true;
}

void read_EEPROM_skipping(uint16_t src_addr,void *dest,int16_t count)
{  
  uint32_t *pSrc = (uint32_t *)(src_addr | 0x8080000);
  int16_t remaining = count;
  int16_t* pDest = dest;
  while (remaining != 0) {
    *pDest = (short)*pSrc;
    pSrc = pSrc + 1;
    remaining = remaining + -1;
    pDest = pDest + 1;
  }
  return;
}

void CreateRadioPacket(RadioPacketTypedef *packet)
{
    uint16_t avg_dmps, gust_dmps, angle_deg;
    get_wind_parameters(&avg_dmps, &gust_dmps, &angle_deg);
    int16_t tempPlus400 = lastTempMeasurement + 400;

    packet->b1 = 0x80;
    memcpy(packet->id, &radio_id, 3);
    packet->light_hi = light >> 8;
    packet->light_lo = light & 0xFF;
    packet->battery = battery;
    packet->hiBits =
        // We're not going to report minVoltage.
        // They're in bit 7 and bit 3.
        // We might need to put something in there to satisfy
        ((gust_dmps & 0x100) >> 2) |
        ((angle_deg & 0x100) >> 3) |
        ((avg_dmps & 0x100) >> 4) |
        ((tempPlus400 & 0x700) >> 8);
    packet->temp_lo = tempPlus400 & 0xFF;
    packet->humidity = lastHumidityMeasurement & 0xFF;
    packet->wind_avg_lo = avg_dmps & 0xFF;
    packet->wind_dir_lo = angle_deg & 0xFF;
    packet->wind_max_lo = gust_dmps & 0xFF;
    packet->uv_index_x10 = uvIndex_x10;
    packet->pressureAndStatus = 0;
    packet->pressure = 0;
    packet->crc1 = crc8_dallas(packet, sizeof(RadioPacketTypedef) - 2, 0x00);
    packet->chkSum = checksum(packet, sizeof(RadioPacketTypedef) - 1);
    RADIO_PRINT("Packet Contents: (%d)\r\n", sizeof(RadioPacketTypedef));
    uint8_t* p = (uint8_t*)packet;
    for (int i = 0; i < sizeof(RadioPacketTypedef); i++)
        RADIO_PRINT("%02x ", p[i]);
    RADIO_PRINT("\r\n");
}

bool processRadio()
{
    uint32_t localTicks = g_rtcTicks;
    if (localTicks - last_tx > radio_tx_interval)
    {
        last_tx = g_rtcTicks;
        RadioPacketTypedef radioPacket;
        CreateRadioPacket(&radioPacket);
        radio_transmit(&radioPacket, sizeof(radioPacket));
        return true;
    }
    return false;
}