/*
 * servo_control.h
 *
 *  Created on: Feb 10, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef SRC_SERVO_CONTROL_H_
#define SRC_SERVO_CONTROL_H_

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"

#include "ST3215/ST3215.h"

#define DEFAULT_SERVO_ID 	1

typedef enum { No_Action = 0, SwitchAction_Stop, SwitchAction_Slowdown } SwitchAction_t;

typedef struct {
	uint16_t RegType;
	uint16_t iRegIndex;
	uint16_t InData;
} QueueCmd_t;

typedef struct {
	// RO EEPROM
	uint16_t fw_main_ver;
	uint16_t fw_sub_ver;
	uint16_t servo_main_ver;
	uint16_t servo_sub_ver;
} Servo_EEPROM_ROData_t;

typedef struct {
	// RO SRAM
	uint16_t actual_pos;
	uint16_t actual_speed;
	uint16_t actual_load;
	uint16_t actual_voltage;
	uint16_t actual_temperature;
	uint16_t actual_servo_status;
	uint16_t actual_current;
} Servo_FLASH_ActualData_t;



typedef struct {

	// RW EPROM
	uint16_t id;
	uint16_t baudrate;
	uint16_t ReturnDelay;
//	uint16_t RespStatus;
	uint16_t MinAngleLim;
	uint16_t MaxAngleLim;
	uint16_t MaxTempLim;
	uint16_t MaxInVLim;
	uint16_t MinInVLim;
	uint16_t MaxTorqueLim;
	uint16_t SettingByte;
	uint16_t UnloadCond;
	uint16_t AlarmLed;
	uint16_t Kp;
	uint16_t Kd;
	uint16_t Ki;
	uint16_t StartupForce;
	uint16_t CW_dead;
	uint16_t CCW_dead;
	uint16_t ProtCurrent;
	uint16_t AngularResolution;
	uint16_t PosOffset;
	uint16_t OpMode;
	uint16_t ProtTorque;
	uint16_t ProtTime;
	uint16_t OverloadTorque;
	uint16_t SpeedCoefP;
	uint16_t OverCurrentTime;
	uint16_t SpeedCoefI;

	// RW SRAM
	uint16_t TorqueTurnOn;
	int16_t cmdAcc;
	int16_t cmdPosition;
	int16_t cmdRunningTime;
	int16_t cmdSpeed;
	int16_t TorqueLimit;
	int16_t UnlockEEPROM;

} ServoCfgData_t;

// указатель на функцию записи параметра
typedef int (*pWriteParam)(ServoST3215* item, int16_t param);

void ServoCtrl_Init(void *phuart);
void ServoCtrl_Poll(void);

void ChangeParametersRequest(uint16_t RegType, uint16_t iRegIndex, uint16_t InData);

#endif /* SRC_SERVO_CONTROL_H_ */
