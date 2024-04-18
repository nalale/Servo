/*
 * ServoControl.c
 *
 *  Created on: Feb 10, 2024
 *      Author: SoftwareEngineer_01
 */

#include <stdint.h>

#include "tools.h"
#include "RingBuffer/ring_buffer.h"

#include "servo_control.h"
#include "sw_sensors.h"
#include "mb_app_data.h"
#include "74HC165.h"

#define SERIAL_FRAME_LEN	22

extern uint16_t usSRegInBuf[];
extern int16_t usSRegHoldBuf[];
extern uint8_t ucSDiscInBuf[];

extern MBRegsTableNote_t MBServoSerialTable[];

ServoST3215 servoItem[32];


typedef enum { STATE_SCAN = 0, STATE_POLL, } ServoCtrlStep_t;
typedef enum { SCAN_WAIT = 0, SCAN_SUCCESS, SCAN_TIMEOUT } ServoCtrlScanResult_t;

static QueueCmd_t BufArray[10];		// Массив для очереди запросов
static RINGBUFF_T InMsgBuf;			// Очередь запросов MB


static uint32_t requestsPeriods[Req_Cnt] = { 1000, 50, 0 };
static ServoCtrlStep_t ServoCtrlStep;
static uint8_t ServoDriveNumber;

static uint32_t sw_handle_ts = 0;

static int8_t actual_data_parse(ServoST3215 *this, uint8_t *pDataBuf);
static int8_t cfg_data_parse(ServoST3215 *this, uint8_t *pDataBuf);
static ServoCtrlScanResult_t ServoCtrl_Scan(ServoST3215 *this, uint8_t Id);
static ServoCtrlScanResult_t ServoCtrl_Poll(ServoST3215 *this);
static void ServoCtrl_Req(ServoST3215 *this, MB_RegType_t RegType, uint16_t iRegIndex, uint16_t InData);

// Сигнал об измении MB Hold Register
void ChangeParametersRequest(uint16_t RegType, uint16_t iRegIndex, uint16_t InData) {
	QueueCmd_t newCmd = {RegType, iRegIndex, InData};
	// поместить параметры в кольцевой буффер команд сервопривода
	RingBuffer_Insert(&InMsgBuf, &newCmd);
}

int8_t ReadCfgParameters(uint16_t RegType, uint16_t iRegIndex, uint16_t *Data) {
	// находим номер привода
	uint8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;
	//iRegIndex += MB_IDX_SERVO_DATA_START;

	uint16_t *array = (uint16_t*)&servoItem[num].RW_MemData;

	*Data = array[iRegIndex];
	return 0;
}

int8_t ReadInputParameters(uint16_t RegType, uint16_t iRegIndex, uint16_t *Data) {

	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;
	//iRegIndex += MB_IDX_SERVO_DATA_START;

	if(num < 0) {
		if(RegType == MB_RegType_InReg)
			*Data = usSRegInBuf[iRegIndex];
		else if(RegType == MB_RegType_DIn)
			*Data = ucSDiscInBuf[iRegIndex];
	}
	else {
		if(RegType == MB_RegType_InReg) {
			uint16_t *array = (uint16_t*)&servoItem[num].RO_MemData;

			*Data = array[iRegIndex];
		} else if(RegType == MB_RegType_DIn) {
			uint16_t *array = (uint16_t*)&servoItem[num].RO_MemData.dins_data;

			*Data = array[iRegIndex];
		}
	}
	return 0;
}

uint8_t ReadDescreteInput(uint16_t RegType, uint16_t iRegIndex, uint16_t *Data) {
	// находим номер привода
	int8_t num = (iRegIndex >> 6) - 1;
	// находим номер параметра
	iRegIndex &= 0x3F;
	//iRegIndex += MB_IDX_SERVO_DATA_START;

	if(num < 0)
		*Data = ucSDiscInBuf[iRegIndex];
	else {
		uint16_t *array = (uint16_t*)&servoItem[num].RO_MemData.dins_data;
		*Data = array[iRegIndex];
	}

	return 0;
}

void ServoCtrl_Init(void *phuart) {
	eMBRegHoldingWriteCB = ChangeParametersRequest;
	eMBRegHoldingReadCB = ReadCfgParameters;
	eMBRegInputReadCB = ReadInputParameters;

	RingBuffer_Init(&InMsgBuf, BufArray, sizeof(BufArray[0]), 10);

	ServoSerialBusInit(phuart);

	ServoCtrlStep = STATE_SCAN;
}

void ServoCtrl_Process() {
	static uint16_t servo_id = 0;
	static uint16_t servo_num = 0;
	ServoCtrlScanResult_t result;

	switch(ServoCtrlStep) {

	case STATE_SCAN:
	{
		result = ServoCtrl_Scan(&servoItem[servo_num], servo_id);

		if(result == SCAN_SUCCESS) {
			servoST3215_init(&servoItem[servo_num], servo_id, NULL);
			servo_num++;
			servo_id++;
		} else if(result == SCAN_TIMEOUT) {
			servo_id++;
		}

		// если прошли все ID у всех 32х приводов
		if(servo_id > 255 || servo_num > 31) {
			ServoDriveNumber = servo_num;
			servo_num = 0;
			ServoCtrlStep = STATE_POLL;
		}
	}
		break;
	case STATE_POLL: {
		QueueCmd_t newCmd;


		result = ServoCtrl_Poll(&servoItem[servo_num]);

		if(result != SCAN_WAIT) {
			if(RingBuffer_Pop(&InMsgBuf, &newCmd)) {
				// номера регистров для серво занимают 64 (1 << 6) позиции.
				// т.е смещением на 6, находим какому из приводов команда
				// после этого оставляем только младшие 63 бита указывающиие, какой именно регистр
				uint8_t num = (newCmd.iRegIndex >> 6) - 1;
				uint16_t regIdx = (newCmd.iRegIndex & 0x3F) + 64;

				//newCmd.iRegIndex &= 0x3F;
				//newCmd.iRegIndex += 64;

				ServoCtrl_Req(&servoItem[num], (MB_RegType_t)newCmd.RegType, regIdx, newCmd.InData);
			}
			else if(++servo_num >= ServoDriveNumber)
				servo_num = 0;
		}

		break;
	}
	}
}

// Возвращает 0, если опрос в процессе
// Возвращает 1, если результат опроса успешный
// Возвращает -1, если результат опроса неуспешный
static ServoCtrlScanResult_t ServoCtrl_Scan(ServoST3215 *this, uint8_t Id) {
	ServoEventType_t Event;

	if(ServoSerialEventGet(&Event) == true) {
		switch(Event) {
		case SERVO_READY:
			this->servoId = Id;
			servoST3215_ping(this);
			break;
		case SERVO_PACKET_RECEIVED: {
			uint8_t ServoID = UINT8_MAX;
			uint8_t ServoStatus = UINT8_MAX;

			// обработать принятые пакеты
			ServoSerialPacketProcessed();

			ServoSerialIdGet(&ServoID);
			ServoSerialStatusGet(&ServoStatus);

			if(ServoID == Id) {
				this->errorState = ServoStatus;
				this->doesExist = 1;

				return SCAN_SUCCESS;
			}
			else
				return SCAN_TIMEOUT;
		}
			break;
		case SERVO_CMD_EXECUTE:
		case SERVO_PACKET_SENT:
			// проверять таймаут приема пакетов
			if(ServoSerialTimeoutCheck()) {
				ServoSerialWaitingExpired();
				return SCAN_TIMEOUT;
			}
			break;
		}
	}

	return SCAN_WAIT;
}

static void ServoCtrl_Req(ServoST3215 *this, MB_RegType_t RegType, uint16_t iRegIndex, uint16_t InData) { //QueueCmd_t *newCmd) {
	ServoEventType_t Event;

/*	uint16_t iRegIndex = newCmd->iRegIndex;
	uint16_t RegType = newCmd->RegType;
	uint16_t InData = newCmd->InData;
*/
	if(ServoSerialEventGet(&Event) == true) {
		switch(Event) {
		case SERVO_READY: {
			MBRegsTableNote_t *holdReg;

			if(iRegIndex == MB_IDX_RW_TARGET_POS) {

				// проверить условия концевиков, если возврат ошибки, не выполнять команду
				if(sw_sens_check_condition(this, InData) == ERROR)
					break;
			}

			// выбрать нужную функцию записи и передать параметр
			if(RegType == MB_RegType_HoldReg)
				holdReg = app_mb_note_find(iRegIndex);
			else if(RegType == MB_RegType_Coil)
				holdReg = app_mb_coil_note_find(iRegIndex);

			// проверяем указатель
			// сделать поиск по номеру регистра MB, а не индекс
			if(holdReg->pWriteParam != NULL) {
				holdReg->pWriteParam(this, (int16_t)InData);

				// Обновить данные EEPROM
				this->readSRAM = 0;
				this->currentRequest = Req_Write;
			}
		}

			break;

		default:
			break;
		}
	}
}

static ServoCtrlScanResult_t ServoCtrl_Poll(ServoST3215 *this) {
	uint8_t Instruction = 0;
	uint8_t ServoID = UINT8_MAX;
	uint8_t ServoStatus = UINT8_MAX;
	uint8_t DataLen;
	uint8_t *pDataBuf = NULL;
	ServoCtrlScanResult_t result;


	ServoEventType_t Event;
	if(ServoSerialEventGet(&Event) == true) {
		switch(Event) {
		case SERVO_READY:

			if(++(this->currentRequest) >= Req_Write)
				this->currentRequest = Req_Ping;

			result = SCAN_SUCCESS;

			if(msTimer_DiffFrom(this->requestsStamps[this->currentRequest]) >= requestsPeriods[this->currentRequest]) {
				this->requestsStamps[this->currentRequest] = msTimer_Now();

				if(this->currentRequest == Req_Ping)
					servoST3215_ping(this);
				else if(this->currentRequest == Req_Poll) {
					// при инициализация считываем EEPROM, далее обновляем данные SRAM
					if(!this->readSRAM) {
						readServoMemory(this->servoId, SMS_STS_WB_MAJ_VER, EEPROM_MEM_SIZE); //SMS_STS_WB_MAJ_VER, SERIAL_FRAME_LEN); //
					} else {
						readServoMemory(this->servoId, SMS_STS_PRESENT_POSITION_L, POLL_BUFFER_SIZE);
					}
				}
				else if(this->currentRequest == Req_Write) {

				}

				result = SCAN_WAIT;
			}

			break;
		case SERVO_PACKET_RECEIVED:
			// обработать принятые пакеты
			ServoSerialPacketProcessed();

			ServoSerialPacketGet(&DataLen, &pDataBuf);
			ServoSerialIdGet(&ServoID);
			ServoSerialStatusGet(&ServoStatus);

			Instruction = this->currentRequest + 1;
			result = SCAN_SUCCESS;
			break;
		case SERVO_CMD_EXECUTE:
		case SERVO_PACKET_SENT:
			// проверять таймаут приема пакетов
			if(ServoSerialTimeoutCheck()) {
				ServoSerialWaitingExpired();
				result = SCAN_TIMEOUT;
			}
			else
				result = SCAN_WAIT;
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
		if(this->readSRAM) {
			actual_data_parse(this, pDataBuf);
		} else {
			cfg_data_parse(this, pDataBuf);
			this->readSRAM = 1;
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

	if(msTimer_DiffFrom(sw_handle_ts) >= 200) {
		sw_handle_ts = HAL_GetTick();

		// установить флаг Online
		if(this->doesExist)
			ucSDiscInBuf[MB_IDX_ACTUAL_SERVO_STATES] |= 1 << 0;
		else
			ucSDiscInBuf[MB_IDX_ACTUAL_SERVO_STATES] &= ~(1 << 0);

		// обработка состояния концевиков
		sw_sens_handle(this);
	}

	return result;
}


static int8_t actual_data_parse(ServoST3215 *this, uint8_t *pDataBuf) {
	// Next, parse the values
	// Value: Position
	//MBRegsTableNote_t *sram_data = app_mb_inreg_note_find(MB_IDX_ACTUAL_POS);
	Servo_FLASH_ActualData_t *buf = &this->RO_MemData.flash_data; //(Servo_FLASH_ActualData_t *)sram_data->pMBRegValue;

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

static int8_t cfg_data_parse(ServoST3215 *this, uint8_t *pDataBuf) {
	// RO SRAM Data
	//MBRegsTableNote_t *sram_data = app_mb_inreg_note_find(MB_IDX_FW_MAIN_VER);
	Servo_EEPROM_ROData_t *inRegs = &this->RO_MemData.eeprom_data;//(Servo_EEPROM_ROData_t *)sram_data->pMBRegValue;

	inRegs->fw_main_ver = pDataBuf[SMS_STS_WB_MAJ_VER];
	inRegs->fw_sub_ver = pDataBuf[SMS_STS_WB_MIN_VER];
	inRegs->servo_main_ver = pDataBuf[SMS_STS_MODEL_H];
	inRegs->servo_sub_ver = pDataBuf[SMS_STS_MODEL_L];

	// RW EEPROM Data
	//MBRegsTableNote_t *ee_data = app_mb_note_find(MB_IDX_SERVO_DATA_START);
	Servo_EEPROM_CfgData_t *buf = &this->RW_MemData.eeprom_data;//(Servo_EEPROM_CfgData_t *)ee_data->pMBRegValue;

	//MBRegsTableNote_t *ro_data = app_mb_note_find(MB_IDX_RW_TORQUE_SW);
	Servo_FLASH_CmdData_t *rw_buf = &this->RW_MemData.flash_data;//(Servo_FLASH_CmdData_t *)ro_data->pMBRegValue;

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

	rw_buf->cmdAcc = pDataBuf[SMS_STS_ACC];

	highByte = pDataBuf[SMS_STS_GOAL_POSITION_H];
	lowByte = pDataBuf[SMS_STS_GOAL_POSITION_L];
	rw_buf->cmdPosition = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_GOAL_SPEED_H];
	lowByte = pDataBuf[SMS_STS_GOAL_SPEED_L];
	rw_buf->cmdSpeed = parse16BitUInt(highByte, lowByte);

	highByte = pDataBuf[SMS_STS_TORQUE_LIMIT_H];
	lowByte = pDataBuf[SMS_STS_TORQUE_LIMIT_L];
	rw_buf->TorqueLimit = parse16BitUInt(highByte, lowByte);
	return 0;
}
