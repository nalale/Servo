/*
 * app_cfg_data.h
 *
 *  Created on: Mar 17, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_APP_CFG_DATA_H_
#define INC_APP_CFG_DATA_H_
/*
typedef struct {
	uint8_t sw_sensor_1_num;
	uint8_t sw_sensor_2_num;
	uint8_t sw_sensor_3_num;
	uint8_t sw_sensor_4_num;
	uint8_t sw_sensor_1_action;
	uint8_t sw_sensor_2_action;
	uint8_t sw_sensor_3_action;
	uint8_t sw_sensor_4_action;
	uint8_t sw_sensor_act_acc;
	uint8_t sw_sensor_act_speed;
} switch_servo_t;
*/

#define CFG_DATA_LEN	64

typedef struct {
	uint16_t CRC_16;

	uint8_t *data;

} app_cfg_t;


#define APP_CFG_FLASH_ADDRESS	0x0801FC00
#define APP_CFG_WORD_NUM		sizeof(app_cfg_t) / sizeof(int32_t)

uint8_t app_cfg_save(uint8_t* cfg_buf, uint16_t len);//app_cfg_t *app_cfg);
uint8_t app_cfg_load(uint8_t* cfg_buf, uint16_t len);//app_cfg_t *app_cfg);

#endif /* INC_APP_CFG_DATA_H_ */
