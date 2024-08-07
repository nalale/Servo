/*
 * mb_app_data.c
 *
 *  Created on: Mar 16, 2024
 *      Author: SoftwareEngineer_01
 */

#include "mb_app_data.h"
#include "app_cfg_data.h"


#define SERVO_MEM_NOT_MAP	{ NULL, SMS_STS_NOT_MAP, 0 }

int16_t usSRegInBuf[MB_INDEG_DEV_DATA_LEN];
int16_t usSRegHoldBuf[MB_IDX_RW_UNIT_DATA_LEN];
uint8_t	ucSCoilBuf[S_COIL_NCOILS]; //UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]
uint8_t	ucSDiscInBuf[MB_DIN_DEV_DATA_LEN];

extern ServoST3215 servoItem[32];
static QueueCmd_t BufArray[10];		// Массив для очереди запросов
RINGBUFF_T InMsgBuf;				// Очередь запросов MB
static 	uint32_t mbProc_ts = 0;

int8_t mb_read_data(uint16_t RegType, uint16_t iRegIndex, uint16_t *Data);
void mb_write_data_req(uint16_t RegType, uint16_t iRegIndex, uint16_t InData);


// отображение запросов MB в ServoSerial
MBRegsTableNote_t MBInRegsTable[] =
{
	{ Note_Type_DeviceState, MB_INREG_DEV_FW_VER_MAJ, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_FW_VER_MIN, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_HW_VER_MAJ, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_HW_VER_MIN, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SERVO_NUM, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 1, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 2, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 3, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 4, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 5, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 6, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 7, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 8, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 9, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 10, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 11, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 12, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 13, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 14, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 15, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 16, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 17, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 18, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 19, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 20, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 21, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 22, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 23, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 24, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 25, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 26, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 27, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 28, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 29, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_1_SERVO_ID + 30, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_DeviceState, MB_INREG_DEV_SLOT_32_SERVO_ID, 0, SERVO_MEM_NOT_MAP },

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
MBRegsTableNote_t MBHoldRegsTable[] =//MBServoSerialTable[] =
{
	{ Note_Type_Cfg, MB_IDX_CFG_ADDRESS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_MD_BD,  0, SERVO_MEM_NOT_MAP },
	{ Note_Type_Cfg, MB_IDX_CFG_SERVO_BD, 0, SERVO_MEM_NOT_MAP },

	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_ID, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_3_POS, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_1_SPEED, 0, SERVO_MEM_NOT_MAP },
	{ Note_Type_GroupCtrl, MB_IDX_RW_GROUP_2_SPEED, 0, SERVO_MEM_NOT_MAP },
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
};


void app_mb_data_init() {
	// Load App Info
	usSRegInBuf[MB_INREG_DEV_FW_VER_MAJ] = FW_VER_MAJ;
	usSRegInBuf[MB_INREG_DEV_FW_VER_MIN] = FW_VER_MIN;
	usSRegInBuf[MB_INREG_DEV_HW_VER_MAJ] = HW_VER_MAJ;
	usSRegInBuf[MB_INREG_DEV_HW_VER_MIN] = HW_VER_MIN;

	// инициализация массива указателей на данные
	// HoldRegs
	for(uint8_t cnt = 0; cnt < MB_HOLDREGS_CNT; cnt++) {
		if(MBHoldRegsTable[cnt].MBRegNum < MB_IDX_RW_UNIT_DATA_LEN)
			MBHoldRegsTable[cnt].pMBRegValue = &usSRegHoldBuf[cnt];
		else {
			int16_t *pData = (int16_t*)&servoItem[0].RW_MemData;
			MBHoldRegsTable[cnt].pMBRegValue = &(pData[cnt - MB_IDX_RW_UNIT_DATA_LEN]);
		}
	}

	// InRegs
	for(uint8_t cnt = 0; cnt < MB_INREGS_CNT; cnt++) {
		if(MBInRegsTable[cnt].MBRegNum < MB_INDEG_DEV_DATA_LEN)
			MBInRegsTable[cnt].pMBRegValue = &usSRegInBuf[cnt];
		else {
			int16_t *pData = (int16_t*)&servoItem[0].RO_MemData;
			MBInRegsTable[cnt].pMBRegValue = &(pData[cnt - MB_INDEG_DEV_DATA_LEN]);
		}
	}

	// Coils
	for(uint8_t cnt = 0; cnt < MB_COILS_CNT; cnt++)
		MBCoilsTable[cnt].pMBRegValue = (int16_t*)&ucSCoilBuf[cnt];

	for(uint8_t cnt = 0; cnt < MB_DIN_CNT; cnt++) {
		if(MBDInTable[cnt].MBRegNum < MB_DIN_DEV_DATA_LEN)
			MBDInTable[cnt].pMBRegValue = (int16_t*)&ucSDiscInBuf[cnt];
		else {
			int16_t *pData = (int16_t*)&servoItem[0].RO_MemData.dins_data;
			MBInRegsTable[cnt].pMBRegValue = &(pData[cnt - MB_DIN_DEV_DATA_LEN]);
		}
	}

	// Инициализация данных конфига устройства
	for(uint8_t cnt = 0, cfg_offset = 0; cnt < MB_HOLDREGS_CNT; cnt++) {

		if(MBHoldRegsTable[cnt].NoteType == Note_Type_Cfg) {
			/*if(cnt < MB_IDX_DEV_CFG_DATA_LEN)
				MBServoSerialTable[cnt].pMBRegValue = (int16_t*)&cfg_array[cfg_offset];
			else {

			}
			 */
			cfg_offset++;
		}
	}

	RingBuffer_Init(&InMsgBuf, BufArray, sizeof(BufArray[0]), 10);

	eMBRegHoldingWriteCB = mb_write_data_req;
	eMBRegInputReadCB = mb_read_data;
}

void app_mb_cmd_proc(void) {
	if(msTimer_DiffFrom(mbProc_ts) > 50) {

		  uint16_t _storeCfgReq = 0;
		  if(mb_read_data(MB_RegType_Coil, MB_IDX_COIL_FLASH_WRITE, &_storeCfgReq)) {
			  app_cfg_data_store();
			  ucSCoilBuf[MB_IDX_COIL_FLASH_WRITE] = 0;
		  }

		  mbProc_ts = HAL_GetTick();
	  }


}

uint8_t app_mb_holdregs_req_get(QueueCmd_t *out) {
	if(RingBuffer_Pop(&InMsgBuf, out) > 0) {
		return SUCCESS;
	}

	out = NULL;
	return ERROR;
}

// Сигнал об измении MB Hold Register
void mb_write_data_req(uint16_t RegType, uint16_t iRegIndex, uint16_t InData) {


	uint8_t num = (iRegIndex >> 6) - 1;			// номер слота с приводом
	uint16_t regIdx = (iRegIndex & 0x3F);		// номер регистра в приводе

	// TODO: для настройки группового управления и настройки платы
	if(num < 0)
		return;

	// Если команда назначается для исполнения сервоприводом,
	// поместить параметры в кольцевой буффер команд сервопривода
	regIdx += MB_IDX_SERVO_DATA_START;
	if((regIdx >= MB_IDX_SERVO_DATA_START) && (regIdx < MB_IDX_CFG_SW_START)){
		QueueCmd_t newCmd = {RegType, iRegIndex, InData};
		RingBuffer_Insert(&InMsgBuf, &newCmd);
	}
	// Если команда для настройки непосредственно платы, то записать сразу
	else if(regIdx >= MB_IDX_CFG_SW_START) {
		uint16_t pAddr = regIdx - MB_IDX_SERVO_DATA_START;
		int16_t *pData = (int16_t*)&servoItem[num].RW_MemData;
		pData[pAddr] = InData;
	}
}


int8_t mb_read_data(uint16_t RegType, uint16_t iRegIndex, uint16_t *Data) {

	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;
	uint16_t *array = NULL;

	if(num < 0) {
		if(RegType == MB_RegType_HoldReg)
			array = (uint16_t*)&usSRegHoldBuf[0];
		else if(RegType == MB_RegType_InReg)
			array = (uint16_t*)&usSRegInBuf[0];
		else if(RegType == MB_RegType_DIn)
			array = (uint16_t*)&ucSDiscInBuf[0];
		else if(RegType == MB_RegType_Coil)
			array = (uint16_t*)&ucSCoilBuf[0];
		else
			return 1;
	}
	else {
		if(RegType == MB_RegType_HoldReg)
			array = (uint16_t*)&servoItem[num].RW_MemData;
		else if(RegType == MB_RegType_InReg)
			array = (uint16_t*)&servoItem[num].RO_MemData;
		else if(RegType == MB_RegType_DIn) {
			array = (uint16_t*)&servoItem[num].RO_MemData.dins_data;

		}
		else
			return 1;
	}

	*Data = array[iRegIndex];
	return 0;
}

//--- HoldRegs Funcs ---------------------/
MBRegsTableNote_t *app_mb_holdreg_find(uint16_t iRegIndex) {
	MBRegsTableNote_t *res = NULL;

	if(app_mb_holdreg_exists(iRegIndex) == SUCCESS) {
		for(uint8_t cnt = 0; cnt < MB_HOLDREGS_CNT; cnt++) {
			if(MBHoldRegsTable[cnt].MBRegNum == iRegIndex){
				res = &MBHoldRegsTable[cnt];
				break;
			}
		}
	}

	return res;
}

ErrorStatus app_mb_holdreg_exists(uint16_t iRegIndex) {

	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;

	// количество слотов не больше 32х
	if(num > 32)
		return ERROR;

	// регистры устройства
	if(num == -1 && iRegIndex >= MB_IDX_RW_UNIT_DATA_LEN)
		return ERROR;

	// регистры приводов
	if(num >= 0 && iRegIndex >= MB_SERVO_DATA_LEN)
		return ERROR;


	return SUCCESS;

}

//--- InRegs Funcs ---------------------/
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

ErrorStatus app_mb_inreg_exists(uint16_t iRegIndex) {

	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;

	// количество слотов не больше 32х
	if(num > 32)
		return ERROR;

	// регистры устройства
	if(num == -1 && iRegIndex >= MB_INDEG_DEV_DATA_LEN)
		return ERROR;

	// регистры приводов
	if(num >= 0 && iRegIndex >= MB_IDX_RO_CNT)
		return ERROR;


	return SUCCESS;
}

uint8_t device_data_update(uint16_t regNum, int16_t value) {

	if(regNum < MB_INDEG_DEV_DATA_LEN) {
		usSRegInBuf[regNum] = value;
		return SUCCESS;
	}
	else
		return ERROR;
}

//--- CoilRegs Funcs ---------------------/
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

ErrorStatus app_mb_coil_exists(uint16_t iRegIndex) {

	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;

	// количество слотов не больше 32х
	if(num > 32)
		return ERROR;

	// регистры устройства
	if(num == -1 && iRegIndex >= MB_COIL_DEV_DATA_LEN)
		return ERROR;

	// регистры приводов
	if(num >= 0 && iRegIndex >= MB_COILS_DATA_LEN)
		return ERROR;


	return SUCCESS;
}

//--- DescreteInRegs Funcs ---------------------/
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
