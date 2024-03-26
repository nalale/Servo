/*
 * tools.c
 *
 *  Created on: Mar 7, 2024
 *      Author: SoftwareEngineer_01
 */

#include <stdint.h>
#include "stm32f1xx_hal.h"

uint32_t msTimer_Now() {

	return HAL_GetTick();
}

uint32_t msTimer_DiffFrom(uint32_t last_ms) {

	uint32_t ms_now = msTimer_Now();

	return (ms_now >= last_ms)?
		 ms_now - last_ms:
		(0xFFFFFFFF - last_ms) + ms_now;
}

/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned short Crc16(unsigned char *pcBlock, unsigned short len)
{
    unsigned short crc = 0xFFFF;
    unsigned char i;

    while (len--)
    {
        crc ^= *pcBlock++ << 8;

        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
