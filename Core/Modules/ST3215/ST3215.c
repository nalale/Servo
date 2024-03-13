/*
 * st3215.c
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "ST3215.h"
#include "servoserial.h"
#include "portservoserial.h"

#define PARAMETERS_DATA_LEN		8

uint8_t parameters[PARAMETERS_DATA_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

void servoST3215_init(ServoST3215 *this, uint8_t id, void *dHUART, uint8_t *ActualDataBuf) {
  this->servoId = id;
  this->errorState = 0;       // Initialize to no error state
  this->doesExist = 0;         // By default, doesn't exist until it's found by ping()

  this->pActualData = ActualDataBuf;
  // Load blank values in variables
/*  this->load = 0;
  this->currentSpeed = 0;
  this->voltage = 0;
  this->current = 0;
  this->currentPosition = 0;
  this->mode = 0;
  this->temp = 0;*/
  this->isMoving = 0;

  ServoSerialBusInit(dHUART);

  //ServoSerialBusStart();
}

void servoST3215_in_buf_set(ServoST3215 *this, uint8_t *ActualDataBuf) {
	this->pActualData = ActualDataBuf;
}

// Returns true if this servo exists, false otherwise;
bool servoST3215_exists(ServoST3215 *this) {
  if (this->doesExist == 1) return true;
  return false;
}

/*
 *  Accessors
 */

int servoST3215_ping(ServoST3215 *this) {
   // First, send the write (ping packets have no parameters)
  const uint8_t numBytesToWrite = 0;
  uint8_t parameters[numBytesToWrite];
  sendServoPacket(this->servoId, INST_PING, (uint8_t*)parameters, numBytesToWrite);
  this->doesExist = 0;


  // Then, read the response
  /*  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];

  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);
  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    this->doesExist = 0;
    return -1;
  }

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    //Serial.printf("ERROR: SERVO RETURNED NON-ZERO WORKING STATUS (%02X)", readBuffer[5]);
    this->errorState = readBuffer[5];
    this->doesExist = 1;
    return -1;
  }

  // Success
  this->doesExist = 1;
  */
  return 1;
}

/*
 *  Setters
 */
int servoST3215_WriteID(ServoST3215 *this, int32_t NewID) {
	parameters[0] = SMS_STS_ID;
	parameters[1] = (uint8_t)NewID;

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 2);
	//servoST3215_LockEprom(this, true);
	return 1;
}
int servoST3215_WriteBaud(ServoST3215 *this, int32_t Baud);

// The min angle is a value between -30719 and 30719
int servoST3215_WriteMinAngleLimit(ServoST3215 *this, int32_t MinAngle) {

	if (MinAngle > 30719)
		MinAngle = 30719;
	else if (MinAngle < -30719)
		MinAngle = -30719;

	parameters[0] = SMS_STS_MIN_ANGLE_LIMIT_L;
	packU16toButter(MinAngle, &parameters[1]);

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);
	//servoST3215_LockEprom(this, true);
	return 1;
}

// The max angle is a value between -30719 and 30719
int servoST3215_WriteMaxAngleLimit(ServoST3215 *this, int32_t MaxAngle) {
	if (MaxAngle > 30719)
		MaxAngle = 30719;
	else if (MaxAngle < -30719)
		MaxAngle = -30719;

	parameters[0] = SMS_STS_MAX_ANGLE_LIMIT_L;
	packU16toButter(MaxAngle, &parameters[1]);

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);
	//servoST3215_LockEprom(this, true);
	return 1;
}

// The min and max angle are values between -30719 and 30719
int servoST3215_WriteMinMaxAngleLimit(ServoST3215 *this, int16_t MinAngle, int16_t MaxAngle) {
	if (MinAngle > 30719) MinAngle = 30719;
	else if (MinAngle < -30719) MinAngle = -30719;

	if (MaxAngle > 30719) MaxAngle = 30719;
	else if (MaxAngle < -30719) MaxAngle = -30719;

	parameters[0] = SMS_STS_MAX_ANGLE_LIMIT_L;

	packU16toButter(MinAngle, &parameters[1]);
	packU16toButter(MaxAngle, &parameters[3]);

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 5);
	//servoST3215_LockEprom(this, true);

	return 1;
}

// Change torque in the SRAM and EPROM, The torque limit is a value between 1 and 1000
int servoST3215_WriteTorqueLimit(ServoST3215 *this, int32_t TorqueLimit) {
	if(TorqueLimit >= 0 && TorqueLimit <= 1000) {
		parameters[0] = SMS_STS_MAX_TORQUE_LIMIT_L;
		packU16toButter(TorqueLimit, &parameters[1]);

		//servoST3215_LockEprom(this, false);
		sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);
		//servoST3215_LockEprom(this, true);
		return 1;
	} else {
		return -1; // Torque limit out of range
	}
}
/* Сhange control mode. Mode values set as follow:
0: Position control
1: Wheel mode / Speed control closed loop speed
2: PWM mode open loop speed
3: Step mode
*/
int servoST3215_WriteMode(ServoST3215 *this, int32_t Mode) {
	parameters[0] = SMS_STS_MODE;
	parameters[1] = Mode;

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 2);
	//servoST3215_LockEprom(this, true);

	return 1;
}

// Change overload current value between 0 and 255
int servoST3215_WriteOverloadCurrent(ServoST3215 *this, int32_t Current) {
	parameters[0] = SMS_STS_OVERLOAD_CURRENT_L;
	packU16toButter(Current, &parameters[1]);

	//servoST3215_LockEprom(this, false);
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);
	//servoST3215_LockEprom(this, true);
	return 1;
}

// Torque or Release the servo motor
// Enable 1: turn on the servo motor, holding its current position
// Enable 0: turn off its motor
int servoST3215_EnableTorque(ServoST3215 *this, int32_t Enable) {
	parameters[0] = SMS_STS_TORQUE_ENABLE;
	parameters[1] = Enable;

	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 2);

	return 1;
}
// Move the servo to a given position
int servoST3215_WriteAcc(ServoST3215 *this, int32_t acceleration) {//, uint16_t speed, uint8_t acceleration) {

	parameters[0] = SMS_STS_ACC;
	// Acceleration
	parameters[1] = acceleration;

	// Send the write
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 2);//8);

	return 1;
}

int servoST3215_WritePosition(ServoST3215 *this, int32_t newPosition) {
	parameters[0] = SMS_STS_GOAL_POSITION_L;

	if(newPosition < 0){
		newPosition = -newPosition;
		newPosition |= (1<<15);
	}

	// Position
	packU16toButter(newPosition, &parameters[1]);
	// Send the write
	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);
	return 1;
}

int servoST3215_WriteTargetSpeed(ServoST3215 *this, int32_t Speed) {
	if(Speed < 0) {
		Speed = -Speed;
		Speed |= (1<<15);
	}

	parameters[0] = SMS_STS_GOAL_SPEED_L;
	packU16toButter(Speed, &parameters[1]);

	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);

	return 1;
}

int servoST3215_WriteTorqueLim(ServoST3215 *this, int32_t Lim) {

	parameters[0] = SMS_STS_TORQUE_LIMIT_L;
	packU16toButter(Lim, &parameters[1]);

	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 3);

	return 1;
}

int servoST3215_RegWriteTargetPosition(ServoST3215 *this, int16_t Position, uint16_t Speed, uint8_t ACC) { return -1;} //Задать положение сервопривода асинхронно(RegWriteAction)
void servoST3215_SyncWriteTargetPositions(ServoST3215 *item[], ServoST3215 *itemN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]){ ;}//Задать несколько положений синхронно


int servoST3215_LockEprom(ServoST3215 *this, int32_t Enable) {
	parameters[0] = SMS_STS_LOCK;
	parameters[1] = Enable > 0;

	sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, 2);

	return 1;
}

int servoST3215_CalibrationOfs(ServoST3215 *this){ return -1;}

int servoST3215_ReadPos(ServoST3215 *item);
int servoST3215_ReadSpeed(ServoST3215 *item);
int servoST3215_ReadLoad(ServoST3215 *item);
int servoST3215_ReadVoltage(ServoST3215 *item);
int servoST3215_ReadTemper(ServoST3215 *item);
int servoST3215_ReadMove(ServoST3215 *item);
int servoST3215_ReadCurrent(ServoST3215 *item);




// Stop the servo motor.
// This is accomplished by releasing it, then quickly torquing it to hold in the current position
/*void servoST3215_stopServo(ServoST3215 *this) {
  servoST3215_releaseServo(this);
  //delay(2);
  servoST3215_torqueServo(this);
}*/

