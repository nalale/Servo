/*
 * st3215.h
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdbool.h>

#ifndef MODULES_ST3215_ST3215_H_
#define MODULES_ST3215_ST3215_H_

#include "ST3215/servoserial.h"

// Register map (from the Waveshare driver)
// EPROM Read Only
#define SMS_STS_WB_MAJ_VER 0
#define SMS_STS_WB_MIN_VER 1
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

// EPROM Read Write
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_DELAY_TIME_RETURN 7
#define SMS_STS_LEVEL_RETURN 8
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_MAX_TEMP_LIMIT 13
#define SMS_STS_MAX_INPUT_VOLT 14
#define SMS_STS_MIN_INPUT_VOLT 15
#define SMS_STS_MAX_TORQUE_LIMIT_L 16
#define SMS_STS_MAX_TORQUE_LIMIT_H 17
#define SMS_STS_SETTING_BYTE 18
#define SMS_STS_UNLOAD_COND 19
#define SMS_STS_ALARM_LED 20
#define SMS_STS_P_COEF 21
#define SMS_STS_D_COEF 22
#define SMS_STS_I_COEF 23
#define SMS_STS_STARTUP_FORCE_L 24
#define SMS_STS_STARTUP_FORCE_H 25
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OVERLOAD_CURRENT_L 28
#define SMS_STS_OVERLOAD_CURRENT_H 29
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

// SRAM Read Write
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

// SRAM Read Only
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_PRESENT_STATUS 65
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

// Poll method: Update the internal state variables by reading their values from the servo
#define POLL_ZERO_OFFSET 	SMS_STS_PRESENT_POSITION_L
#define POLL_BUFFER_SIZE 	SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + 1
#define EEPROM_MEM_SIZE 	SMS_STS_MODE - SMS_STS_WB_MAJ_VER + 1
#define PACKET_HEADER_SIZE  5
#define PACKET_FOOTER_SIZE  1

enum baud {BAUD_1000000, BAUD_500000, BAUD_250000, BAUD_128000, BAUD_115200, BAUD_76800, BAUD_57600, BAUD_38400, NBBAUD};
enum mode {POSITION, SPEED, PWM, STEP, NBMODE};

typedef struct {
	// RO EEPROM
	uint16_t fw_main_ver;
	uint16_t fw_sub_ver;
	uint16_t servo_main_ver;
	uint16_t servo_sub_ver;

	// RO SRAM
	uint16_t actual_pos;
	uint16_t actual_speed;
	uint16_t actual_load;
	uint16_t actual_voltage;
	uint16_t actual_temperature;
	uint16_t actual_servo_status;
	uint16_t actual_current;
} ServoActualData_t;



typedef struct {

	// RW EPROM
	uint16_t id;
	uint16_t baudrate;
	uint16_t ReturnDelay;
	uint16_t RespStatus;
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

typedef struct {

    // Accessors (updated by poll() )
    uint8_t   servoId;
    //ServoActualData_t *pActualData;
    uint8_t *pActualData;


    uint8_t  isMoving;

    uint8_t   doesExist;      // Does this servo exist (i.e. respond to ping?)
    uint8_t   errorState;     // Current error state



    uint8_t dataUpdated;
} ServoST3215;


extern void servoST3215_init(ServoST3215 *this, uint8_t id, void *dHUART, uint8_t *ActualDataBuf);
// Does the servo exist (i.e. respond to ping?)
extern bool servoST3215_exists(ServoST3215 *item);
// Ping the servo
extern int servoST3215_ping(ServoST3215 *item);

extern void servoST3215_in_buf_set(ServoST3215 *this, uint8_t *ActualDataBuf);

int servoST3215_WriteID(ServoST3215 *item, int32_t NewID);
int servoST3215_WriteBaud(ServoST3215 *item, int32_t Baud);
int servoST3215_WriteMinAngleLimit(ServoST3215 *item, int32_t MinAngle);
int servoST3215_WriteMaxAngleLimit(ServoST3215 *item, int32_t MaxAngle);
int servoST3215_WriteMinMaxAngleLimit(ServoST3215 *item, int16_t MinAngle, int16_t MaxAngle);
int servoST3215_WriteTorqueLimit(ServoST3215 *item, int32_t TorqueLimit);
int servoST3215_WriteMode(ServoST3215 *item, int32_t Mode);
int servoST3215_WriteOverloadCurrent(ServoST3215 *item, int32_t Current);

int servoST3215_EnableTorque(ServoST3215 *item, int32_t Enable);
int servoST3215_WriteAcc(ServoST3215 *this, int32_t acceleration);
int servoST3215_WritePosition(ServoST3215 *this, int32_t newPosition);
int servoST3215_WriteTargetSpeed(ServoST3215 *this, int32_t Speed);
int servoST3215_WritePWM(ServoST3215 *this, int32_t newPWM);
int servoST3215_WriteTorqueLim(ServoST3215 *this, int32_t Lim);

//int servoST3215_WriteTargetPosition(ServoST3215 *item, int16_t Position, uint16_t Speed, uint8_t ACC);//Задать положение сервопривода
int servoST3215_RegWriteTargetPosition(ServoST3215 *item, int16_t Position, uint16_t Speed, uint8_t ACC);//Задать положение сервопривода асинхронно(RegWriteAction)
void servoST3215_SyncWriteTargetPositions(ServoST3215 *item[], ServoST3215 *itemN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);//Задать несколько положений синхронно


int servoST3215_LockEprom(ServoST3215 *item, int32_t Enable);
int servoST3215_CalibrationOfs(ServoST3215 *item);

int servoST3215_ReadPos(ServoST3215 *item);
int servoST3215_ReadSpeed(ServoST3215 *item);
int servoST3215_ReadLoad(ServoST3215 *item);
int servoST3215_ReadVoltage(ServoST3215 *item);
int servoST3215_ReadTemper(ServoST3215 *item);
int servoST3215_ReadMove(ServoST3215 *item);
int servoST3215_ReadCurrent(ServoST3215 *item);


#endif /* MODULES_ST3215_ST3215_H_ */
