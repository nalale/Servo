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

typedef enum { Req_Ping = 0, Req_Poll, Req_Write, Req_Cnt} Requests_t;

#define DEFAULT_SERVO_ID 1

// отображение запросов MB в ServoSerial
static MBServoSerialNode_t MBServoSerialTable[MB_IDX_RW_CNT] =
{
	{ MB_Type_HoldReg, MB_IDX_RW_SERVO_ID, servoST3215_WriteID, SMS_STS_ID, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_BAUDRATE, NULL, SMS_STS_BAUD_RATE, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_RET_DELAY, NULL, SMS_STS_DELAY_TIME_RETURN, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_RESP_STATUS, NULL, SMS_STS_LEVEL_RETURN, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_MIN_ANGLE_LIM, servoST3215_WriteMinAngleLimit, SMS_STS_MIN_ANGLE_LIMIT_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_MAX_ANGLE_LIM, servoST3215_WriteMaxAngleLimit, SMS_STS_MAX_ANGLE_LIMIT_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_MAX_TEMP_LIM, NULL, SMS_STS_MAX_TEMP_LIMIT, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_MAX_IN_VOLTAGE, NULL, SMS_STS_MAX_INPUT_VOLT, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_MIN_IN_VOLTAGE, NULL, SMS_STS_MIN_INPUT_VOLT, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_MAX_TORQUE, servoST3215_WriteTorqueLimit, SMS_STS_MAX_TORQUE_LIMIT_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_PHASE, NULL, SMS_STS_SETTING_BYTE, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_UNLOAD_CONDTITION, NULL, SMS_STS_UNLOAD_COND, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_LED_ALARM_CONDITION, NULL, SMS_STS_ALARM_LED, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_P_COEF, NULL, SMS_STS_P_COEF, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_D_COEF, NULL, SMS_STS_D_COEF, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_I_COEF, NULL, SMS_STS_I_COEF, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_MIN_START_FORCE, NULL, SMS_STS_STARTUP_FORCE_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_CW_INSENS_AREA, NULL, SMS_STS_CW_DEAD, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_CCW_INSENS_AREA, NULL, SMS_STS_CCW_DEAD, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_PROT_CURRENT, servoST3215_WriteOverloadCurrent, SMS_STS_OVERLOAD_CURRENT_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_ANGULAR_RESOLUTION, NULL, SMS_STS_BAUD_RATE, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_POS_CORRECTION, NULL, SMS_STS_OFS_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_OPERATE_MODE, servoST3215_WriteMode, SMS_STS_MODE, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_PROT_TORQUE, NULL, 0, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_PROT_TIME, NULL, 0, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_OVERLOAD_TORQUE, NULL, 0, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_SPEED_P_COEF, NULL, 0, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_OVERCURR_PROT_TIME, NULL, 0, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_SPEED_I_COEF, NULL, 0, 1 },

	{ MB_Type_HoldReg, MB_IDX_RW_TORQUE_SW, NULL, SMS_STS_TORQUE_ENABLE, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_ACC, servoST3215_WriteAcc, SMS_STS_ACC, 1 },
	{ MB_Type_HoldReg, MB_IDX_RW_TARGET_POS, servoST3215_WritePosition, SMS_STS_GOAL_POSITION_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_RUNNING_TIME, NULL, SMS_STS_GOAL_TIME_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_RUNNING_SPEED, servoST3215_WriteTargetSpeed, SMS_STS_GOAL_SPEED_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_TORQUE_LIM, servoST3215_WriteTorqueLim, SMS_STS_TORQUE_LIMIT_L, 2 },
	{ MB_Type_HoldReg, MB_IDX_RW_LOCK_MARK, servoST3215_LockEprom, SMS_STS_LOCK, 1 },
};

static ServoST3215 servoItem[32];
static uint16_t BufArray[10];
static RINGBUFF_T InMsgBuf;
static uint32_t requestsPeriods[Req_Cnt] = { 1000, 100, 0 };
static uint32_t requestsStamps[Req_Cnt];
static uint8_t currentRequest;
static uint8_t readSRAM = 0;	// Флаг, говорящий читать SRAM или EEPROM


extern uint8_t ucSDiscInBuf[];
extern uint16_t usSRegInBuf[];
extern uint16_t usSRegHoldBuf[];

// Сигнал об измении MB Hold Register
static void ChangeParametersRequest(uint16_t iRegIndex, uint16_t usNRegs) {
	// поместить параметры в кольцевой буффер команд сервопривода
	RingBuffer_Insert(&InMsgBuf, &iRegIndex);
}

void ServoCtrl_Init(void *phuart) {
	eMBRegHoldingWriteCB = ChangeParametersRequest;
	RingBuffer_Init(&InMsgBuf, BufArray, sizeof(BufArray[0]), 10);


	// Чтение EEPROM в буффер MBRegHold
	readSRAM = 0;
	servoST3215_init(&servoItem[DEFAULT_SERVO_ID], DEFAULT_SERVO_ID, phuart, (uint8_t*)usSRegHoldBuf);
}

void ServoCtrl_Poll() {
	uint16_t changed_index;
	uint8_t Instruction = 0;
	uint8_t ServoID = UINT8_MAX;
	uint8_t ServoStatus = UINT8_MAX;
	uint8_t DataLen;
	uint8_t *pDataBuf = NULL;
	ServoST3215 *this = &servoItem[DEFAULT_SERVO_ID];

	ServoEventType_t Event;
	if(ServoSerialEventGet(&Event) == true) {
		switch(Event) {
		case SERVO_READY:

			if(++currentRequest >= Req_Write)
				currentRequest = Req_Ping;

			if(RingBuffer_Pop(&InMsgBuf, &changed_index)) {
				currentRequest = Req_Write;
			}

			if(msTimer_DiffFrom(requestsStamps[currentRequest]) >= requestsPeriods[currentRequest]) {
				requestsStamps[currentRequest] = msTimer_Now();

				if(currentRequest == Req_Ping)
					servoST3215_ping(this);
				else if(currentRequest == Req_Poll) {
					// при инициализация считываем EEPROM, далее обновляем данные SRAM
					if(!readSRAM) {
						servoST3215_in_buf_set(this, (uint8_t*)usSRegHoldBuf);
						readServoMemory(this->servoId, SMS_STS_WB_MAJ_VER, EEPROM_MEM_SIZE);
					} else {
						servoST3215_in_buf_set(this, (uint8_t*)usSRegInBuf);
						readServoMemory(this->servoId, SMS_STS_PRESENT_POSITION_L, POLL_BUFFER_SIZE);
					}
				}
				else if(currentRequest == Req_Write) {
					// выбрать нужную функцию записи и передать параметр
					uint16_t param = usSRegHoldBuf[changed_index];

					// проверяем указатель
					// сделать поиск по номеру регистра MB, а не индекс
					if(MBServoSerialTable[changed_index].pWriteParam != NULL) {
						MBServoSerialTable[changed_index].pWriteParam(this, (int32_t)param);

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
		if(readSRAM) {
			// Next, parse the values
			// Value: Position
			ServoActualData_t *buf = (ServoActualData_t *)usSRegInBuf;

			uint8_t highByte = pDataBuf[SMS_STS_PRESENT_POSITION_H - POLL_ZERO_OFFSET];
			uint8_t lowByte = pDataBuf[SMS_STS_PRESENT_POSITION_L - POLL_ZERO_OFFSET];
			buf->actual_pos = parse16BitUInt(highByte, lowByte);

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
		} else {

			// RO SRAM Data
			ServoActualData_t *holdRegs = (ServoActualData_t *)usSRegInBuf;

			holdRegs->fw_main_ver = pDataBuf[SMS_STS_WB_MAJ_VER];
			holdRegs->fw_sub_ver = pDataBuf[SMS_STS_WB_MIN_VER];
			holdRegs->servo_main_ver = pDataBuf[SMS_STS_MODEL_H];
			holdRegs->servo_sub_ver = pDataBuf[SMS_STS_MODEL_L];

			// RW EEPROM Data
			ServoCfgData_t *buf = (ServoCfgData_t *)usSRegHoldBuf;

			buf->id = pDataBuf[SMS_STS_ID];
			buf->baudrate = pDataBuf[SMS_STS_BAUD_RATE];
			buf->ReturnDelay = pDataBuf[SMS_STS_DELAY_TIME_RETURN];
			buf->RespStatus = pDataBuf[SMS_STS_LEVEL_RETURN];

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

			buf->OpMode = pDataBuf[SMS_STS_MODE];

			// Чтение EEPROM завершено
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


	/*
	 *if(servoItem[0].dataUpdated) {
		ucSDiscInBuf[0] = servoItem[0].doesExist;
		ucSDiscInBuf[1] = servoItem[0].isMoving;

		usSRegInBuf[0] = servoItem[0].currentPosition;
		usSRegInBuf[1] = servoItem[0].currentSpeed;
		usSRegInBuf[2] = servoItem[0].load;
		usSRegInBuf[3] = servoItem[0].current;
		usSRegInBuf[4] = servoItem[0].voltage;
		usSRegInBuf[5] = servoItem[0].temp;
		usSRegInBuf[6] = servoItem[0].mode;
	  }

	  if(msTimer_DiffFrom(mb_read_ts) > 10) {
		  servoST3215_setPosition(&servoItem[0], usSRegHoldBuf[0]);
		  servoST3215_setSpeed(&servoItem[0], usSRegHoldBuf[1]);
		  servoST3215_setAcc(&servoItem[0], usSRegHoldBuf[2]);

		  mb_read_ts = HAL_GetTick();
	  }
	 *
	 */


}

void servoST3215_poll(ServoST3215 *this) {




}
