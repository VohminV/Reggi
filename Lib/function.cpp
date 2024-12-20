#include <Arduino.h>
#include "variables.h"

uint8_t ICACHE_RAM_ATTR calc(const uint8_t data)
{
    return crc8tab[data];
}

uint8_t ICACHE_RAM_ATTR calc(const uint8_t *data, uint8_t len, uint8_t crc)
{
    while (len--)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}


void ICACHE_RAM_ATTR initializeCRC8(void)
{
    uint8_t crc;

    for (uint16_t i = 0; i < crclen; i++)
    {
        crc = i;
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        }
        crc8tab[i] = crc & 0xFF;
    }
}