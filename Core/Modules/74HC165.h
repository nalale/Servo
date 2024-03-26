/*
 * 74РС165.h
 *
 *  Created on: Mar 14, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef MODULES_74HC165_H_
#define MODULES_74HC165_H_

typedef enum {
	SwitchSens_Error = 0x0,
	SwitchSens_On = 0x1,
	SwitchSens_Off = 0x2,
	SwitchSens_Empty = 0x3,
} SwitchSensState_t;

void shift_reg_init(void *hspi, uint16_t registers_num);
void shift_reg_latch_data (void);
int8_t shift_reg_data_get(uint16_t sensor_num, SwitchSensState_t* result);
int8_t shift_reg_din_get(uint16_t reg, uint8_t din_num, SwitchSensState_t* result);
int8_t shift_reg_switches_get(uint8_t *sw_array, uint16_t len);


#endif /* MODULES_74HC165_H_ */
