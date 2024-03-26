/*
 * mb_app_data.c
 *
 *  Created on: Mar 16, 2024
 *      Author: SoftwareEngineer_01
 */

#include "mb_app_data.h"
#include "app_cfg_data.h"


#define SERVO_MEM_NOT_MAP	{ NULL, SMS_STS_NOT_MAP, 0 }

int16_t usSRegInBuf[MB_INREGS_CNT];
int16_t usSRegHoldBuf[MB_HOLDREGS_CNT];


uint8_t    ucSCoilBuf[S_COIL_NCOILS]; //UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]
extern uint8_t		ucSDiscInBuf[];


// отображение запросов MB в ServoSerial
MBRegsTableNote_t MBInRegsTable[] =
{
	{ Note_Type_MBReg, MB_INREG_DEV_FW_VER_MAJ, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_INREG_DEV_FW_VER_MIN, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_INREG_DEV_HW_VER_MAJ, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_INREG_DEV_HW_VER_MIN, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_INREG_DEV_SERVO_NUM, 0, SERVO_MEM_NOT_MAP },


	{ Note_Type_MBReg, MB_IDX_FW_MAIN_VER, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_FW_SUB_VER, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_SERVO_MAIN_VER, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_SERVO_SUB_VER, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_SPEED, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_LOAD, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_VOLTAGE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_TEMPERATURE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_SERVO_STATUS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_MBReg, MB_IDX_ACTUAL_CURRENT, 0, SERVO_MEM_NOT_MAP },
};

// отображение запросов MB в ServoSerial
MBRegsTableNote_t MBServoSerialTable[] =
{
	{ Note_Type_Cfg, MB_IDX_CFG_ADDRESS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_MD_BD,  0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SERVO_BD, 0, SERVO_MEM_NOT_MAP },

	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_DUTY, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_SPEED, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_DUTY, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_SPEED, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_DUTY, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_SPEED, 0, SERVO_MEM_NOT_MAP },

	// Начало регистров отображаемых на память Servo
	{ Note_Type_MBReg, MB_IDX_RW_SERVO_ID, 0, { servoST3215_WriteID, (uint16_t)SMS_STS_ID, 1 } },
	{ Note_Type_MBReg, MB_IDX_RW_BAUDRATE, 0, { NULL, SMS_STS_BAUD_RATE, 1 } },
	{ Note_Type_MBReg, MB_IDX_RW_RET_DELAY, 0, { NULL, SMS_STS_DELAY_TIME_RETURN, 1 } },
	{ Note_Type_MBReg, MB_IDX_RW_MIN_ANGLE_LIM, 0, {servoST3215_WriteMinAngleLimit, SMS_STS_MIN_ANGLE_LIMIT_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_MAX_ANGLE_LIM, 0, {servoST3215_WriteMaxAngleLimit, SMS_STS_MAX_ANGLE_LIMIT_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_MAX_TEMP_LIM, 0, {NULL, SMS_STS_MAX_TEMP_LIMIT, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_MAX_IN_VOLTAGE, 0, {NULL, SMS_STS_MAX_INPUT_VOLT, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_MIN_IN_VOLTAGE, 0, {NULL, SMS_STS_MIN_INPUT_VOLT, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_MAX_TORQUE, 0, {servoST3215_WriteTorqueLimit, SMS_STS_MAX_TORQUE_LIMIT_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_PHASE, 0, {NULL, SMS_STS_SETTING_BYTE, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_UNLOAD_CONDTITION, 0, {NULL, SMS_STS_UNLOAD_COND, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_LED_ALARM_CONDITION, 0, {NULL, SMS_STS_ALARM_LED, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_P_COEF, 0, {NULL, SMS_STS_P_COEF, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_D_COEF, 0, {NULL, SMS_STS_D_COEF, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_I_COEF, 0, {NULL, SMS_STS_I_COEF, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_MIN_START_FORCE, 0, {NULL, SMS_STS_STARTUP_FORCE_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_CW_INSENS_AREA, 0, {NULL, SMS_STS_CW_DEAD, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_CCW_INSENS_AREA, 0, { NULL, SMS_STS_CCW_DEAD, 1 }},
	{ Note_Type_MBReg, MB_IDX_RW_PROT_CURRENT, 0, {servoST3215_WriteOverloadCurrent, SMS_STS_OVERLOAD_CURRENT_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_ANGULAR_RESOLUTION, 0, {NULL, SMS_STS_BAUD_RATE, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_POS_CORRECTION, 0, {NULL, SMS_STS_OFS_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_OPERATE_MODE, 0, { servoST3215_WriteMode, SMS_STS_MODE, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_PROT_TORQUE, 0, { NULL, 0, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_PROT_TIME, 0, { NULL, 0, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_OVERLOAD_TORQUE, 0, { NULL, 0, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_SPEED_P_COEF, 0, { NULL, 0, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_OVERCURR_PROT_TIME, 0, { NULL, 0, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_SPEED_I_COEF, 0, { NULL, 0, 1} },

	{ Note_Type_MBReg, MB_IDX_RW_TORQUE_SW, 0, { NULL, SMS_STS_TORQUE_ENABLE, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_ACC, 0, { servoST3215_WriteAcc, SMS_STS_ACC, 1} },
	{ Note_Type_MBReg, MB_IDX_RW_TARGET_POS, 0, { servoST3215_WritePosition, SMS_STS_GOAL_POSITION_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_RUNNING_TIME, 0, { servoST3215_WritePWM, SMS_STS_GOAL_TIME_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_RUNNING_SPEED, 0, { servoST3215_WriteTargetSpeed, SMS_STS_GOAL_SPEED_L, 2} },
	{ Note_Type_MBReg, MB_IDX_RW_TORQUE_LIM, 0, { servoST3215_WriteTorqueLim, SMS_STS_TORQUE_LIMIT_L, 2} },

	// Регистры привязки концевиков к серво
	{ Note_Type_Cfg, MB_IDX_CFG_SW_1_NUM, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_2_NUM, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_3_NUM, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_4_NUM, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_1_ACT, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_2_ACT, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_3_ACT, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SW_4_ACT, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SLOWDOWN_DEACC, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SLOWDOWN_SPEED, 0, SERVO_MEM_NOT_MAP },
};

// отображение запросов MB в ServoSerial
MBRegsTableNote_t MBCoilsTable[] =
{
	{ Note_Type_DeviceCtrl, MB_IDX_COIL_FLASH_WRITE, 0, SERVO_MEM_NOT_MAP },

	{ Note_Type_DeviceCtrl, MB_IDX_COIL_RESP_STATUS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceCtrl, MB_IDX_COIL_LOCK_MARK, 0, { servoST3215_LockEprom, SMS_STS_LOCK, 1} },
};

// отображение запросов MB в ServoSerial
MBRegsTableNote_t MBDInTable[] =
{
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_1_8_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_9_16_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_17_24_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_25_32_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_33_40_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_41_48_STATE, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_DIN_SW_49_50_STATE, 0, SERVO_MEM_NOT_MAP },

	{ Note_Type_DeviceState, MB_IDX_ACTUAL_SERVO_STATES, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_IDX_SERVO_SW_STATES, 0, SERVO_MEM_NOT_MAP },
};


void app_mb_data_init() {

	uint8_t cfg_array[13];
	app_cfg_load(cfg_array, 13);

	// инициализация массива указателей на данные
	for(uint8_t cnt = 0; cnt < MB_HOLDREGS_CNT; cnt++)
		MBServoSerialTable[cnt].pMBRegValue = &usSRegHoldBuf[cnt];

	for(uint8_t cnt = 0; cnt < MB_INREGS_CNT; cnt++)
		MBInRegsTable[cnt].pMBRegValue = &usSRegInBuf[cnt];

	for(uint8_t cnt = 0; cnt < MB_COILS_CNT; cnt++)
		MBCoilsTable[cnt].pMBRegValue = (int16_t*)&ucSCoilBuf[cnt];

	for(uint8_t cnt = 0; cnt < MB_DIN_CNT; cnt++)
		MBDInTable[cnt].pMBRegValue = (int16_t*)&ucSDiscInBuf[cnt];

	// Инициализация данных конфига устройства
	for(uint8_t cnt = 0, cfg_offset = 0; cnt < MB_HOLDREGS_CNT; cnt++) {

		if(MBServoSerialTable[cnt].NoteType == Note_Type_Cfg) {
			*MBServoSerialTable[cnt].pMBRegValue = cfg_array[cfg_offset];
			cfg_offset++;
		}
	}
}

void app_mb_cfg_data_store() {
	// добавить впереди 2 пустые запись для CRC
	uint8_t cfg_array[13 + 2];

	for(uint8_t cnt = 0, cfg_offset = 2; cnt < MB_HOLDREGS_CNT; cnt++) {
		if(MBServoSerialTable[cnt].NoteType == Note_Type_Cfg) {
			cfg_array[cfg_offset] = *MBServoSerialTable[cnt].pMBRegValue;
			cfg_offset++;
		}
	}

	app_cfg_save(cfg_array, 13 + 2);
}

MBRegsTableNote_t *app_mb_note_find(uint16_t regNum) {
	MBRegsTableNote_t *res = NULL;
	for(uint8_t cnt = 0; cnt < MB_HOLDREGS_CNT; cnt++) {
		if(MBServoSerialTable[cnt].MBRegNum == regNum){
			res = &MBServoSerialTable[cnt];
			break;
		}
	}

	return res;
}

uint8_t app_mb_holdreg_exists(uint16_t regNum) {
	// Если регистр больше самого полседнего элемента или
	// находится между регистрами настройки устройства и регистрами памяти серво
	if((regNum >= MB_SERVO_DATA_LEN) || (regNum >= MB_IDX_RW_UNIT_DATA_LEN && regNum < MB_IDX_SERVO_DATA_START))
		return 0;

	return 1;
}

MBRegsTableNote_t *app_mb_inreg_note_find(uint16_t regNum) {
	MBRegsTableNote_t *res = NULL;
	for(uint8_t cnt = 0; cnt < MB_INREGS_CNT; cnt++) {
		if(MBInRegsTable[cnt].MBRegNum == regNum) {
			res = &MBInRegsTable[cnt];
			break;
		}
	}

	return res;
}

uint8_t app_mb_inreg_exists(uint16_t regNum) {
	// Если регистр больше самого полседнего элемента или
	// находится между регистрами настройки устройства и регистрами памяти серво
	if((regNum >= MB_IDX_RO_CNT) || (regNum >= MB_INDEG_DEV_DATA_LEN && regNum < MB_IDX_SERVO_DATA_START))
		return 0;

	return 1;
}

MBRegsTableNote_t *app_mb_coil_note_find(uint16_t regNum) {
	MBRegsTableNote_t *res = NULL;
	for(uint8_t cnt = 0; cnt < MB_COILS_CNT; cnt++) {
		if(MBCoilsTable[cnt].MBRegNum == regNum) {
			res = &MBCoilsTable[cnt];
			break;
		}
	}

	return res;
}

uint8_t app_mb_coil_exists(uint16_t regNum) {
	// Если регистр больше самого полседнего элемента или
	// находится между регистрами настройки устройства и регистрами памяти серво
	if((regNum >= MB_COILS_DATA_LEN) || (regNum >= MB_COIL_DEV_DATA_LEN && regNum < MB_IDX_SERVO_DATA_START))
		return 0;

	return 1;
}


MBRegsTableNote_t *app_mb_din_note_find(uint16_t regNum) {
	MBRegsTableNote_t *res = NULL;

	for(uint8_t cnt = 0; cnt < MB_DIN_CNT; cnt++) {
		if(MBCoilsTable[cnt].MBRegNum == regNum) {
			res = &MBDInTable[cnt];
			break;
		}
	}

	return res;
}

uint8_t app_mb_din_exists(uint16_t regNum, uint16_t bitNum) {
	// Если регистр больше самого полседнего элемента или
	// находится между регистрами настройки устройства и регистрами памяти серво
	if((regNum >= MB_DIN_DATA_LEN) || (regNum >= MB_DIN_DEV_DATA_LEN && regNum < MB_IDX_SERVO_DATA_START))
		return 0;

	// В последнем регистра только 2 датчика
	if(regNum == MB_DIN_DEV_DATA_LEN - 1 && bitNum >= 2)
		return 0;

	return 1;
}
