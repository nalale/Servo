/*
 * 74HC165.c
 *
 *  Created on: Mar 14, 2024
 *      Author: SoftwareEngineer_01
 */

#include <string.h>
#include "main.h"
#include "dwt_stm32_delay.h"

//static SPI_HandleTypeDef *pspi;
static uint16_t reg_num;
static uint8_t RxBuffer[20];


void shift_reg_init(void *hspi, uint16_t registers_num) {
	//pspi = hspi;
	reg_num = registers_num;

	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_SET);
}

void latch_next_data () {
	// Строб сигнала !PL, для захвата данных
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_RESET);
	//DWT_Delay_us(1);
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_SET);

	// Выключение сигнала CLOCK_ENABLE, для передачи в последовательный порт
	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_RESET);

	memset(RxBuffer, 0, reg_num);

	for(uint16_t reg = 0; reg < reg_num; reg++) {
		for (int i = 7; i >= 0; i--) {
				RxBuffer[reg] |= (uint8_t) (HAL_GPIO_ReadPin(SHIFT_DATA_IN_GPIO_Port, SHIFT_DATA_IN_Pin) == GPIO_PIN_SET) << i;
				HAL_GPIO_WritePin(SHIFT_CLK_GPIO_Port, SHIFT_CLK_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SHIFT_CLK_GPIO_Port, SHIFT_CLK_Pin, GPIO_PIN_RESET);
			}
	}
	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);

	//HAL_SPI_Receive_IT(pspi,(uint8_t*)RxBuffer, reg_num);
}

/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(pspi->Instance == hspi->Instance) {


		HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);
	}
}*/
