/*
 * ServoControl.c
 *
 *  Created on: Feb 10, 2024
 *      Author: SoftwareEngineer_01
 */

#include <stdint.h>

#include "tools.h"
#include "RingBuffer/ring_buffer.h"

#include "sw_sensors.h"
#include "servo_control.h"
#include "mb_app_data.h"
#include "74HC165.h"

#define SERIAL_FRAME_LEN	22

typedef enum { Req_Ping = 0, Req_Poll, Req_Write, Req_Cnt} Requests_t;

extern uint16_t usSRegInBuf[];
extern int16_t usSRegHoldBuf[];
extern uint8_t ucSDiscInBuf[];

extern MBRegsTableNote_t MBServoSerialTable[];

static ServoST3215 servoItem[32];


static QueueCmd_t BufArray[10];		// Массив для очереди запросов
static RINGBUFF_T InMsgBuf;			// Очередь запросов MB


static uint32_t requestsPeriods[Req_Cnt] = { 1000, 50, 0 };
static uint32_t requestsStamps[Req_Cnt];
static uint8_t currentRequest;
static uint8_t readSRAM = 0;		// Флаг, указывающий читать SRAM или EEPROM

static uint32_t shreg_read_ts = 0;

static int8_t actual_data_parse(uint8_t *pDataBuf);
static int8_t cfg_data_parse(uint8_t *pDataBuf);


// Сигнал об измении MB Hold Register
void ChangeParametersRequest(uint16_t RegType, uint16_t iRegIndex, uint16_t InData) {
	QueueCmd_t newCmd = {RegType, iRegIndex, InData};
	// поместить параметры в кольцевой буффер команд сервопривода
	RingBuffer_Insert(&InMsgBuf, &newCmd);
}

void ServoCtrl_Init(void *phuart) {
	eMBRegHoldingWriteCB = ChangeParametersRequest;
	RingBuffer_Init(&InMsgBuf, BufArray, sizeof(BufArray[0]), 10);


	// Чтение EEPROM в буффер MBRegHold
	readSRAM = 0;
	servoST3215_init(&servoItem[DEFAULT_SERVO_ID], DEFAULT_SERVO_ID, phuart);
}

void ServoCtrl_Poll() {
	uint8_t Instruction = 0;
	uint8_t ServoID = UINT8_MAX;
	uint8_t ServoStatus = UINT8_MAX;
	uint8_t DataLen;
	uint8_t *pDataBuf = NULL;
	ServoST3215 *this = &servoItem[DEFAULT_SERVO_ID];
	QueueCmd_t newCmd;

	ServoEventType_t Event;
	if(ServoSerialEventGet(&Event) == true) {
		switch(Event) {
		case SERVO_READY:

			if(++currentRequest >= Req_Write)
				currentRequest = Req_Ping;

			if(RingBuffer_Pop(&InMsgBuf, &newCmd)) {
				currentRequest = Req_Write;
			}

			if(msTimer_DiffFrom(requestsStamps[currentRequest]) >= requestsPeriods[currentRequest]) {
				requestsStamps[currentRequest] = msTimer_Now();

				if(currentRequest == Req_Ping)
					servoST3215_ping(this);
				else if(currentRequest == Req_Poll) {
					// при инициализация считываем EEPROM, далее обновляем данные SRAM
					if(!readSRAM) {
						readServoMemory(this->servoId, SMS_STS_WB_MAJ_VER, EEPROM_MEM_SIZE); //SMS_STS_WB_MAJ_VER, SERIAL_FRAME_LEN); //
					} else {
						readServoMemory(this->servoId, SMS_STS_PRESENT_POSITION_L, POLL_BUFFER_SIZE);
					}
				}
				else if(currentRequest == Req_Write) {
					MBRegsTableNote_t *holdReg;

					if(newCmd.iRegIndex == MB_IDX_RW_TARGET_POS) {

						// проверить условия концевиков, если возврат ошибки, не выполнять команду
						if(sw_sens_check_condition(newCmd.InData) == ERROR)
							break;
					}

					// выбрать нужную функцию записи и передать параметр
					if(newCmd.RegType == MB_RegType_HoldReg)
						holdReg = app_mb_note_find(newCmd.iRegIndex);
					else if(newCmd.RegType == MB_RegType_Coil)
						holdReg = app_mb_coil_note_find(newCmd.iRegIndex);

					// проверяем указатель
					// сделать поиск по номеру регистра MB, а не индекс
					if(holdReg->pWriteParam != NULL) {
						holdReg->pWriteParam(this, (int16_t)newCmd.InData);

						// Обновить данные EEPROM
						readSRAM = 0;
					}
				}
			}

			break;
		case SERVO_PACKET_RECEIVED:
			// обработать принятые пакеты
			ServoSerialPacketProcessed();

			ServoSerialPacketGet(&DataLen, &pDataBuf);
			ServoSerialIdGet(&ServoID);
			ServoSerialStatusGet(&ServoStatus);
			this = &servoItem[ServoID];
			Instruction = currentRequest + 1;

			break;
		case SERVO_CMD_EXECUTE:
			// проверять таймаут приема пакетов
			if(ServoSerialTimeoutCheck())
				ServoSerialWaitingExpired();
			break;
		case SERVO_PACKET_SENT:
			break;
		}

	}

	switch(Instruction) {

	case INST_PING: {
		if (ServoStatus != 0) {
			//ERROR: SERVO RETURNED NON-ZERO WORKING STATUS readBuffer[5];
			this->errorState = ServoStatus;
		}
		this->doesExist = 1;
	}
	break;

	case INST_READ: {
		// парсинг данных из RO SRAM или RW EEPROM/SRAM
		if(readSRAM) {
			actual_data_parse(pDataBuf);
		} else {
			cfg_data_parse(pDataBuf);
			readSRAM = 1;
		}

		if (ServoStatus != 0)
			this->errorState = ServoStatus;
	}
	break;

	case INST_WRITE: {
		if (ServoStatus != 0)
			this->errorState = ServoStatus;
	}
	break;
	default:
		break;
	}

	if(msTimer_DiffFrom(shreg_read_ts) >= 200) {
		shreg_read_ts = HAL_GetTick();

		// установить флаг Online
		if(this->doesExist)
			ucSDiscInBuf[MB_IDX_ACTUAL_SERVO_STATES] |= 1 << 0;
		else
			ucSDiscInBuf[MB_IDX_ACTUAL_SERVO_STATES] &= ~(1 << 0);

		// обработка состояния концевиков
		sw_sens_handle();
	}
}


static int8_t actual_data_parse(uint8_t *pDataBuf) {
	// Next, parse the values
	// Value: Position
	MBRegsTableNote_t *sram_data = app_mb_inreg_note_find(MB_IDX_ACTUAL_POS);
	Servo_FLASH_ActualData_t *buf = (Servo_FLASH_ActualData_t *)sram_data->pMBRegValue;

	uint8_t highByte = pDataBuf[SMS_STS_PRESENT_POSITION_H - POLL_ZERO_OFFSET];
	uint8_t lowByte = pDataBuf[SMS_STS_PRESENT_POSITION_L - POLL_ZERO_OFFSET];
	buf->actual_pos = parse16BitInt(highByte, lowByte);

	// Value: Speed
	highByte = pDataBuf[SMS_STS_PRESENT_SPEED_H - POLL_ZERO_OFFSET];
	lowByte = pDataBuf[SMS_STS_PRESENT_SPEED_L - POLL_ZERO_OFFSET];
	buf->actual_speed = parse16BitUInt(highByte, lowByte);

	// Value: Load
	highByte = pDataBuf[SMS_STS_PRESENT_LOAD_H - POLL_ZERO_OFFSET];
	lowByte = pDataBuf[SMS_STS_PRESENT_LOAD_L - POLL_ZERO_OFFSET];
	buf->actual_load = parse16BitUInt(highByte, lowByte);

	// Value: Current
	highByte = pDataBuf[SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET];
	lowByte = pDataBuf[SMS_STS_PRESENT_CURRENT_L - POLL_ZERO_OFFSET];
	buf->actual_current = parse16BitUInt(highByte, lowByte);

	// Value: Voltage
	buf->actual_voltage = pDataBuf[SMS_STS_PRESENT_VOLTAGE - POLL_ZERO_OFFSET];

	// Value: Temperature
	buf->actual_temperature = pDataBuf[SMS_STS_PRESENT_TEMPERATURE - POLL_ZERO_OFFSET];

	buf->actual_servo_status = pDataBuf[SMS_STS_PRESENT_STATUS - POLL_ZERO_OFFSET];

	// Value: isMoving
	/*		  uint8_t isMoving = pDataBuf[SMS_STS_MOVING - POLL_ZERO_OFFSET];
	this->isMoving = isMoving;

	// Value: Mode
	uint8_t mode = pDataBuf[SMS_STS_MODE - POLL_ZERO_OFFSET];
	this->mode = mode;
	*/
	return 0;
}

static int8_t cfg_data_parse(uint8_t *pDataBuf) {
	// RO SRAM Data
	MBRegsTableNote_t *sram_data = app_mb_inreg_note_find(MB_IDX_FW_MAIN_VER);
	Servo_EEPROM_ROData_t *inRegs = (Servo_EEPROM_ROData_t *)sram_data->pMBRegValue;

	inRegs->fw_main_ver = pDataBuf[SMS_STS_WB_MAJ_VER];
	inRegs->fw_sub_ver = pDataBuf[SMS_STS_WB_MIN_VER];
	inRegs->servo_main_ver = pDataBuf[SMS_STS_MODEL_H];
	inRegs->servo_sub_ver = pDataBuf[SMS_STS_MODEL_L];

	// RW EEPROM Data
	MBRegsTableNote_t *ee_data = app_mb_note_find(MB_IDX_SERVO_DATA_START);
	ServoCfgData_t *buf = (ServoCfgData_t *)ee_data->pMBRegValue;

	buf->id = pDataBuf[SMS_STS_ID];
	buf->baudrate = pDataBuf[SMS_STS_BAUD_RATE];
	buf->ReturnDelay = pDataBuf[SMS_STS_DELAY_TIME_RETURN];
	//buf->RespStatus = pDataBuf[SMS_STS_LEVEL_RETURN];

	uint8_t highByte = pDataBuf[SMS_STS_MIN_ANGLE_LIMIT_H];
	uint8_t lowByte = pDataBuf[SMS_STS_MIN_ANGLE_LIMIT_L];
	buf->MinAngleLim = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_MAX_ANGLE_LIMIT_H];
	lowByte = pDataBuf[SMS_STS_MAX_ANGLE_LIMIT_L];
	buf->MaxAngleLim = parse16BitUInt(highByte, lowByte);

	buf->MaxTempLim = pDataBuf[SMS_STS_MAX_TEMP_LIMIT];
	buf->MaxInVLim = pDataBuf[SMS_STS_MAX_INPUT_VOLT];
	buf->MinInVLim = pDataBuf[SMS_STS_MIN_INPUT_VOLT];

	highByte = pDataBuf[SMS_STS_MAX_TORQUE_LIMIT_H];
	lowByte = pDataBuf[SMS_STS_MAX_TORQUE_LIMIT_L];
	buf->MaxTorqueLim = parse16BitUInt(highByte, lowByte);

	buf->SettingByte = pDataBuf[SMS_STS_SETTING_BYTE];
	buf->UnloadCond = pDataBuf[SMS_STS_UNLOAD_COND];
	buf->AlarmLed = pDataBuf[SMS_STS_ALARM_LED];
	buf->Kp = pDataBuf[SMS_STS_P_COEF];
	buf->Kd = pDataBuf[SMS_STS_D_COEF];
	buf->Ki = pDataBuf[SMS_STS_I_COEF];

	highByte = pDataBuf[SMS_STS_STARTUP_FORCE_H];
	lowByte = pDataBuf[SMS_STS_STARTUP_FORCE_L];
	buf->StartupForce = parse16BitUInt(highByte, lowByte);

	buf->CW_dead = pDataBuf[SMS_STS_CW_DEAD];
	buf->CCW_dead = pDataBuf[SMS_STS_CCW_DEAD];

	highByte = pDataBuf[SMS_STS_OVERLOAD_CURRENT_H];
	lowByte = pDataBuf[SMS_STS_OVERLOAD_CURRENT_L];
	buf->ProtTorque = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_OFS_H];
	lowByte = pDataBuf[SMS_STS_OFS_L];
	buf->PosOffset = parse16BitUInt(highByte, lowByte);

	buf->AngularResolution = pDataBuf[SMS_STS_ANG_RESOLUTION];

	buf->OpMode = pDataBuf[SMS_STS_MODE];

	buf->cmdAcc = pDataBuf[SMS_STS_ACC];

	highByte = pDataBuf[SMS_STS_GOAL_POSITION_H];
	lowByte = pDataBuf[SMS_STS_GOAL_POSITION_L];
	buf->cmdPosition = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_GOAL_SPEED_H];
	lowByte = pDataBuf[SMS_STS_GOAL_SPEED_L];
	buf->cmdSpeed = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_TORQUE_LIMIT_H];
	lowByte = pDataBuf[SMS_STS_TORQUE_LIMIT_L];
	buf->TorqueLimit = parse16BitUInt(highByte, lowByte);
	return 0;
}
