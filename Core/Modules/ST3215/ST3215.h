/*
 * st3215.h
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#ifndef MODULES_ST3215_ST3215_H_
#define MODULES_ST3215_ST3215_H_

#include <stdbool.h>
#include "ST3215/servoserial.h"

#define DEV_RO_VALUES_CNT	11

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

typedef enum { Req_Ping = 0, Req_Poll, Req_Write, Req_Cnt} Requests_t;

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
	// RW SRAM
	uint16_t TorqueTurnOn;
	int16_t cmdAcc;
	int16_t cmdPosition;
	int16_t cmdRunningTime;
	int16_t cmdSpeed;
	int16_t TorqueLimit;
	int16_t UnlockEEPROM;
} Servo_FLASH_CmdData_t;

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
} Servo_EEPROM_CfgData_t;

typedef struct {
	uint16_t	CFG_SW_1_NUM;		// датчик при вращении в - сторону
	uint16_t	CFG_SW_2_NUM;		// датчик при вращении в - сторону
	uint16_t	CFG_SW_3_NUM;		// датчик при вращении в + сторону
	uint16_t	CFG_SW_4_NUM;		// датчик при вращении в + сторону
	uint16_t	CFG_SW_1_ACT;
	uint16_t	CFG_SW_2_ACT;
	uint16_t	CFG_SW_3_ACT;
	uint16_t	CFG_SW_4_ACT;
	uint16_t	CFG_SLOWDOWN_DEACC;
	uint16_t	CFG_SLOWDOWN_SPEED;
} SwitchSens_Cfg_t;

typedef struct {
	uint16_t SwSensStates[4];
} SwitchSens_Data_t;

typedef union {
	uint16_t DiscreteStates;
	uint16_t
		IsOnline	: 1,
		IsMoving	: 1,
		StateSw1	: 1,
		StateSw2	: 1,
		StateSw3	: 1,
		StateSw4	: 1;

} Servo_RO_Flags_t;

typedef struct {
	Servo_EEPROM_ROData_t eeprom_data;
	Servo_FLASH_ActualData_t flash_data;
	Servo_RO_Flags_t dins_data;
	//SwitchSens_Data_t sw_sens_data;

} RO_MemData_t;

typedef struct {
	Servo_EEPROM_CfgData_t eeprom_data;
	Servo_FLASH_CmdData_t flash_data;
	SwitchSens_Cfg_t sw_sens_cfg;
} RW_MemData_t;




typedef struct {

    // Accessors (updated by poll() )
    uint8_t   servoId;
    //ServoActualData_t *pActualData;
    uint8_t *pActualData;

    // RO Memory data
    RO_MemData_t RO_MemData;

    // RW Memory data
    RW_MemData_t RW_MemData;


    uint8_t  isMoving;

    uint8_t   doesExist;      // Does this servo exist (i.e. respond to ping?)
    uint8_t   errorState;     // Current error state
    uint8_t dataUpdated;


    uint32_t requestsStamps[Req_Cnt];
    uint8_t currentRequest;
    uint8_t readSRAM;		// Флаг, указывающий читать SRAM или EEPROM

    uint8_t sw_neg_stop;
    uint8_t sw_neg_slow;
    uint8_t sw_pos_stop;
    uint8_t sw_pos_slow;
    uint32_t normal_speed;

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
