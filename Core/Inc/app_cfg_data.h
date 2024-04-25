/*
 * app_cfg_data.h
 *
 *  Created on: Mar 17, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef INC_APP_CFG_DATA_H_
#define INC_APP_CFG_DATA_H_


#define FW_VER_MAJ	0
#define FW_VER_MIN	2

#define HW_VER_MAJ	0
#define HW_VER_MIN	1

#define CFG_MAX_DATA_LEN	512

typedef struct {
	uint16_t CRC_16;
	uint16_t MB_ADDRESS;
	uint16_t MB_BAUDRATE;
	uint16_t SERVO_SERIAL_BAUDRATE;
} app_device_settings_t;

typedef struct {
	uint8_t crc_8;
	uint8_t servo_id;
	uint8_t sw_left_1;		// датчик при вращении в - сторону
	uint8_t sw_left_2;		// датчик при вращении в - сторону
	uint8_t sw_right_1;		// датчик при вращении в + сторону
	uint8_t sw_right_2;		// датчик при вращении в + сторону
	uint8_t sw_left_1_act;
	uint8_t sw_left_2_act;
	uint8_t sw_right_1_act;
	uint8_t sw_right_2_act;
	uint8_t sw_act_acc;
	uint8_t sw_act_speed;

} app_servo_cfg_t;

typedef struct {
	app_device_settings_t devSettings;
	app_servo_cfg_t servoCfg[32];
} app_cfg_data_t;

typedef struct {
	uint16_t CRC_16;

	union {
		uint8_t *data;
		app_cfg_data_t *cfg;
	};

} app_cfg_t;


#define APP_CFG_FLASH_ADDRESS	0x0801FC00
#define SERVO_CFG_FLASH_ADDRESS	(APP_CFG_FLASH_ADDRESS + sizeof(app_device_settings_t))


uint8_t app_cfg_save(uint8_t* cfg_buf, uint16_t len);//app_cfg_t *app_cfg);
uint8_t app_cfg_load(uint8_t* cfg_buf, uint16_t len);//app_cfg_t *app_cfg);

#endif /* INC_APP_CFG_DATA_H_ */
