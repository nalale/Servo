/*
 * sw_sensors.h
 *
 *  Created on: Mar 25, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_SW_SENSORS_H_
#define INC_SW_SENSORS_H_

void sw_sens_handle(void);

uint8_t sw_sens_check_condition(uint16_t inputParam);

uint8_t sw_neg_stop_get();
uint8_t sw_neg_slow_get();
uint8_t sw_pos_stop_get();
uint8_t sw_pos_slow_get();
uint8_t sw_normal_speed_get();

#endif /* INC_SW_SENSORS_H_ */
