/*
 * 74HC165.c
 *
 *  Created on: Mar 14, 2024
 *      Author: SoftwareEngineer_01
 */

#include <string.h>
#include "main.h"
#include "dwt_stm32_delay.h"
#include "74HC165.h"

//static SPI_HandleTypeDef *pspi;
static uint16_t reg_num;
static uint8_t RxBuffer[13];
static uint32_t shreg_read_ts = 0;

static void shift_reg_latch_data (void);
static int8_t shift_reg_switches_get(uint8_t *sw_array, uint16_t len);

void shift_reg_init(void *hspi, uint16_t sens_num) {
	//pspi = hspi;
	reg_num = sens_num / 4 + 1;

	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_SET);
}

void shift_reg_proccess(uint8_t *sw_array, uint16_t bits_len) {
	if(msTimer_DiffFrom(shreg_read_ts) >= 200) {
		shreg_read_ts = HAL_GetTick();

		shift_reg_latch_data();
		shift_reg_switches_get(sw_array, bits_len);
	}
}

static void shift_reg_latch_data () {
	// Строб сигнала !PL, для захвата данных
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_RESET);

	for(uint16_t reg = 0; reg < reg_num; reg++)
		RxBuffer[reg] = 0;

	//DWT_Delay_us(1);
	HAL_GPIO_WritePin(LATCH_DATA_GPIO_Port, LATCH_DATA_Pin, GPIO_PIN_SET);

	// Выключение сигнала CLOCK_ENABLE, для передачи в последовательный порт
	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_RESET);

	//memset(RxBuffer, 0, reg_num);

	for(uint16_t reg = 0; reg < reg_num; reg++) {
		for (int i = 7; i >= 0; i--) {
				RxBuffer[(reg_num - 1) - reg] |= (uint8_t) (HAL_GPIO_ReadPin(SHIFT_DATA_IN_GPIO_Port, SHIFT_DATA_IN_Pin) == GPIO_PIN_SET) << i;
				HAL_GPIO_WritePin(SHIFT_CLK_GPIO_Port, SHIFT_CLK_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(SHIFT_CLK_GPIO_Port, SHIFT_CLK_Pin, GPIO_PIN_RESET);
			}
	}
	HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);

	//HAL_SPI_Receive_IT(pspi,(uint8_t*)RxBuffer, reg_num);
}

// получить значение подключенного концевика sensor_num
int8_t shift_reg_data_get(uint16_t sensor_num, SwitchSensState_t* result) {

	// отнимаем 1, т.к счет датчиков с 1
	sensor_num -= 1;

	// один регистр сдвига обрабатывает 4 датчика
	// номер датчика делим на 4, чтобы найти какой регистр его обрабатывает
	uint8_t reg = sensor_num >> 2;
	// ищем какой вход регистра занимает датчик
	uint8_t din_reg = (sensor_num % 4) << 1;

	*result = 0x3 & (RxBuffer[reg] >> din_reg);

	return 1;
}

// получить вход din_num регистра reg
int8_t shift_reg_din_get(uint16_t reg, uint8_t din_num, SwitchSensState_t* result) {
	if(reg > reg_num || din_num > 4)
		return -1;

	*result = 0x3 & (RxBuffer[reg] >> (din_num * 2));

	return 1;
}

// функция перевода состояние датчиков в битовый массив
/*
 * @param: sw_array - битовый массив состояний
 * 			bits_len - количество бит
 */
static int8_t shift_reg_switches_get(uint8_t *sw_array, uint16_t bits_len) {
	uint16_t bit_pos = 0;
	// пройти по всем сдвиговым регистрам
	for(uint8_t reg = 0, bit = 0, sw_cnt = 0; (reg < reg_num) && (bit_pos < bits_len); reg++) {
		// обработать входы по 2
		for(uint8_t din = 0; din < 4; din++, bit++, bit_pos++) {
			uint8_t pos = 0;
			shift_reg_din_get(reg, din, &pos);
			//shift_reg_data_get(sens_num + 1, &pos);

			if(bit == 8) {
				bit = 0;
				sw_cnt += 1;
			}

			if(pos == SwitchSens_On)
				sw_array[sw_cnt] |= 1 << bit;
			else
				sw_array[sw_cnt] &= ~(1 << bit);


		}
	}

	return 1;
}

/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(pspi->Instance == hspi->Instance) {


		HAL_GPIO_WritePin(CLOCK_ENABLE_GPIO_Port, CLOCK_ENABLE_Pin, GPIO_PIN_SET);
	}
}*/
