/*
 * mb_app_data.h
 *
 *  Created on: Mar 16, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef SRC_MB_APP_DATA_H_
#define SRC_MB_APP_DATA_H_

#include "servo_control.h"
#include "RingBuffer/ring_buffer.h"

typedef struct {
	uint16_t RegType;
	uint16_t iRegIndex;
	uint16_t InData;
} QueueCmd_t;

typedef enum {
	MB_RegType_HoldReg = 0,
	MB_RegType_InReg,
	MB_RegType_Coil,
	MB_RegType_DIn,
}MB_RegType_t;

typedef enum {
	Note_Type_MBReg = 0,		// запись для отображения MB на Serial
	Note_Type_Cfg,				// запись конфига самого устройства
	Note_Type_GroupCtrl,		// запись управления группой
	Note_Type_DeviceCtrl,		// управление устройством
	Note_Type_DeviceState,
	Note_Type_DeviceStateArray,
}Note_Type_t;


typedef struct {
	Note_Type_t NoteType;

	uint16_t MBRegNum;
	int16_t* pMBRegValue;		// ссылка на элемент массива с данными MB регистра

	struct {
		pWriteParam pWriteParam;
		ServoMemRegister_t ServoRegNum;
		uint16_t ParamsLen;
	};
} MBRegsTableNote_t;

void app_mb_data_init();
void app_mb_cmd_proc(void);

uint8_t app_mb_holdregs_req_get(QueueCmd_t *out);

MBRegsTableNote_t *app_mb_holdreg_find(uint16_t regNum);
MBRegsTableNote_t *app_mb_inreg_note_find(uint16_t regNum);
MBRegsTableNote_t *app_mb_coil_note_find(uint16_t regNum);

ErrorStatus app_mb_holdreg_exists(uint16_t iRegIndex);
ErrorStatus app_mb_inreg_exists(uint16_t iRegIndex);
ErrorStatus app_mb_coil_exists(uint16_t regNum);
uint8_t app_mb_din_exists(uint16_t regNum, uint16_t bitNum);

// Обновить данные, относящиеся к устройству
uint8_t device_data_update(uint16_t regNum, int16_t value);

extern RINGBUFF_T InMsgBuf;


#endif /* SRC_MB_APP_DATA_H_ */
