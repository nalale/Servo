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
