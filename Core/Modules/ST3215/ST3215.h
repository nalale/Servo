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
typedef enum {
	// EPROM Read Only
	SMS_STS_WB_MAJ_VER = 0,
	SMS_STS_WB_MIN_VER = 1,
	SMS_STS_MODEL_L = 3,
	SMS_STS_MODEL_H = 4,

	// EPROM Read Write

	SMS_STS_ID = 5,
	SMS_STS_BAUD_RATE = 6,
	SMS_STS_DELAY_TIME_RETURN = 7,
	SMS_STS_LEVEL_RETURN = 8,
	SMS_STS_MIN_ANGLE_LIMIT_L = 9,
	SMS_STS_MIN_ANGLE_LIMIT_H = 10,
	SMS_STS_MAX_ANGLE_LIMIT_L = 11,
	SMS_STS_MAX_ANGLE_LIMIT_H = 12,
	SMS_STS_MAX_TEMP_LIMIT = 13,
	SMS_STS_MAX_INPUT_VOLT = 14,
	SMS_STS_MIN_INPUT_VOLT = 15,
	SMS_STS_MAX_TORQUE_LIMIT_L = 16,
	SMS_STS_MAX_TORQUE_LIMIT_H = 17,
	SMS_STS_SETTING_BYTE = 18,
	SMS_STS_UNLOAD_COND = 19,
	SMS_STS_ALARM_LED = 20,
	SMS_STS_P_COEF = 21,
	SMS_STS_D_COEF = 22,
	SMS_STS_I_COEF = 23,
	SMS_STS_STARTUP_FORCE_L = 24,
	SMS_STS_STARTUP_FORCE_H = 25,
	SMS_STS_CW_DEAD = 26,
	SMS_STS_CCW_DEAD = 27,
	SMS_STS_OVERLOAD_CURRENT_L = 28,
	SMS_STS_OVERLOAD_CURRENT_H = 29,
	SMS_STS_ANG_RESOLUTION = 30,
	SMS_STS_OFS_L = 31,
	SMS_STS_OFS_H = 32,
	SMS_STS_MODE = 33,

	// SRAM Read Write
	SMS_STS_TORQUE_ENABLE = 40,
	SMS_STS_ACC = 41,
	SMS_STS_GOAL_POSITION_L = 42,
	SMS_STS_GOAL_POSITION_H = 43,
	SMS_STS_GOAL_TIME_L = 44,
	SMS_STS_GOAL_TIME_H = 45,
	SMS_STS_GOAL_SPEED_L = 46,
	SMS_STS_GOAL_SPEED_H = 47,
	SMS_STS_TORQUE_LIMIT_L = 48,
	SMS_STS_TORQUE_LIMIT_H = 49,
	SMS_STS_LOCK = 55,

	// SRAM Read Only
	SMS_STS_PRESENT_POSITION_L = 56,
	SMS_STS_PRESENT_POSITION_H = 57,
	SMS_STS_PRESENT_SPEED_L = 58,
	SMS_STS_PRESENT_SPEED_H = 59,
	SMS_STS_PRESENT_LOAD_L = 60,
	SMS_STS_PRESENT_LOAD_H = 61,
	SMS_STS_PRESENT_VOLTAGE = 62,
	SMS_STS_PRESENT_TEMPERATURE = 63,
	SMS_STS_ASYNC_WRITE_FLAG = 64,
	SMS_STS_PRESENT_STATUS = 65,
	SMS_STS_MOVING = 66,
	SMS_STS_PRESENT_CURRENT_L = 69,
	SMS_STS_PRESENT_CURRENT_H = 70,

	SMS_STS_NOT_MAP = -1
} ServoMemRegister_t;

// Poll method: Update the internal state variables by reading their values from the servo
#define POLL_ZERO_OFFSET 	SMS_STS_PRESENT_POSITION_L
#define POLL_BUFFER_SIZE 	SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + 1
#define EEPROM_MEM_SIZE 	/*SMS_STS_MODE*/ SMS_STS_TORQUE_LIMIT_H - SMS_STS_WB_MAJ_VER + 1
#define PACKET_HEADER_SIZE  5
#define PACKET_FOOTER_SIZE  1

enum baud {BAUD_1000000, BAUD_500000, BAUD_250000, BAUD_128000, BAUD_115200, BAUD_76800, BAUD_57600, BAUD_38400, NBBAUD};
enum mode {POSITION, SPEED, PWM, STEP, NBMODE};

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


extern void servoST3215_init(ServoST3215 *this, uint8_t id, void *dHUART);
// Does the servo exist (i.e. respond to ping?)
extern bool servoST3215_exists(ServoST3215 *item);
// Ping the servo
extern int servoST3215_ping(ServoST3215 *item);

int servoST3215_WriteID(ServoST3215 *item, int16_t NewID);
int servoST3215_WriteBaud(ServoST3215 *item, int16_t Baud);
int servoST3215_WriteMinAngleLimit(ServoST3215 *item, int16_t MinAngle);
int servoST3215_WriteMaxAngleLimit(ServoST3215 *item, int16_t MaxAngle);
int servoST3215_WriteMinMaxAngleLimit(ServoST3215 *item, int16_t MinAngle, int16_t MaxAngle);
int servoST3215_WriteTorqueLimit(ServoST3215 *item, int16_t TorqueLimit);
int servoST3215_WriteMode(ServoST3215 *item, int16_t Mode);
int servoST3215_WriteOverloadCurrent(ServoST3215 *item, int16_t Current);

int servoST3215_EnableTorque(ServoST3215 *item, int16_t Enable);
int servoST3215_WriteAcc(ServoST3215 *this, int16_t acceleration);
int servoST3215_WritePosition(ServoST3215 *this, int16_t newPosition);
int servoST3215_WriteTargetSpeed(ServoST3215 *this, int16_t Speed);
int servoST3215_WritePWM(ServoST3215 *this, int16_t newPWM);
int servoST3215_WriteTorqueLim(ServoST3215 *this, int16_t Lim);

//int servoST3215_WriteTargetPosition(ServoST3215 *item, int16_t Position, uint16_t Speed, uint8_t ACC);//Задать положение сервопривода
int servoST3215_RegWriteTargetPosition(ServoST3215 *item, int16_t Position, uint16_t Speed, uint8_t ACC);//Задать положение сервопривода асинхронно(RegWriteAction)
void servoST3215_SyncWriteTargetPositions(ServoST3215 *item[], ServoST3215 *itemN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);//Задать несколько положений синхронно


int servoST3215_LockEprom(ServoST3215 *item, int16_t Enable);
int servoST3215_CalibrationOfs(ServoST3215 *item);

int servoST3215_ReadPos(ServoST3215 *item);
int servoST3215_ReadSpeed(ServoST3215 *item);
int servoST3215_ReadLoad(ServoST3215 *item);
int servoST3215_ReadVoltage(ServoST3215 *item);
int servoST3215_ReadTemper(ServoST3215 *item);
int servoST3215_ReadMove(ServoST3215 *item);
int servoST3215_ReadCurrent(ServoST3215 *item);


#endif /* MODULES_ST3215_ST3215_H_ */
