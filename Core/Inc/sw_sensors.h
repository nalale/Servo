/*
 * sw_sensors.h
 *
 *  Created on: Mar 25, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_SW_SENSORS_H_
#define INC_SW_SENSORS_H_

void sw_sens_handle(ServoST3215 *servo);

uint8_t sw_sens_check_condition(ServoST3215 *servo, uint16_t inputParam);


#endif /* INC_SW_SENSORS_H_ */
