/*
 * st3215.c
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdint.h>

#include "ST3215.h"
#include "servoserial.h"

#define PARAMETERS_DATA_LEN		8



void servoST3215_init(ServoST3215 *this, uint8_t id, void *dHUART) {
  this->servoId = id;
  this->errorState = 0;       // Initialize to no error state
  this->doesExist = 0;         // By default, doesn't exist until it's found by ping()

  // Load blank values in variables
  this->load = 0;
  this->currentSpeed = 0;
  this->voltage = 0;
  this->current = 0;
  this->currentPosition = 0;
  this->mode = 0;
  this->temp = 0;
  this->isMoving = 0;

  initializeServoSerialBus(dHUART);
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

  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
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
  return 1;
}

// Poll method: Update the internal state variables by reading their values from the servo
#define POLL_ZERO_OFFSET SMS_STS_PRESENT_POSITION_L
#define POLL_BUFFER_SIZE SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + 1
#define PACKET_HEADER_SIZE  5
#define PACKET_FOOTER_SIZE  1

void servoST3215_poll(ServoST3215 *this) {

  // First, read the servo memory
  uint8_t servoMemoryPacketBuffer[POLL_BUFFER_SIZE + PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE];    // Add 6 for the header/checksum
  int success = readServoMemory(this->servoId, SMS_STS_PRESENT_POSITION_L, POLL_BUFFER_SIZE, (uint8_t *)servoMemoryPacketBuffer);
  if (!success)
    this->errorState = 1;


  /*
  // Display read buffer
  Serial.print("READ BUFFER: ");
  for (int i=0; i<sizeof(servoMemoryPacketBuffer); i++) {
    printHex(servoMemoryPacketBuffer[i]);
    Serial.print(" ");
  }
  Serial.println("");
  */

  // Next, parse the values

  // Value: Position
  uint8_t highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_POSITION_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint8_t lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_POSITION_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t position = parse16BitUInt(highByte, lowByte);
  this->currentPosition = position;

  // Value: Speed
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_SPEED_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_SPEED_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t speed = parse16BitUInt(highByte, lowByte);
  this->currentSpeed = speed;

  // Value: Load
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_LOAD_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_LOAD_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t load = parse16BitUInt(highByte, lowByte);
  this->load = load;

  // Value: Current
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_CURRENT_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t current = parse16BitUInt(highByte, lowByte);
  this->current = current;

  // Value: Voltage
  uint8_t voltage = servoMemoryPacketBuffer[SMS_STS_PRESENT_VOLTAGE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->voltage = voltage;

  // Value: Temperature
  uint8_t temperature = servoMemoryPacketBuffer[SMS_STS_PRESENT_TEMPERATURE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->temp = temperature;

  // Value: isMoving
  uint8_t isMoving = servoMemoryPacketBuffer[SMS_STS_MOVING - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->isMoving = isMoving;

  // Value: Mode
  uint8_t mode = servoMemoryPacketBuffer[SMS_STS_MODE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->mode = mode;
}

/*
 *  Setters
 */

// Move the servo to a given position
int servoST3215_movePosition(ServoST3215 *this, uint16_t newPosition, uint16_t speed, uint8_t acceleration) {
  // First, send the write
  //const uint8_t numBytesToWrite = 8;
  uint8_t parameters[PARAMETERS_DATA_LEN] = {SMS_STS_ACC, 0, 0, 0, 0, 0, 0, 0};

  // Acceleration
  parameters[1] = acceleration;
  // Position
  packU16toButter(newPosition, &parameters[2]);
  // Something in between position and speed that should stay at 0?
  // Speed
  packU16toButter(speed, &parameters[6]);
  // Send the write
  sendServoPacket(this->servoId, INST_WRITE, (uint8_t*)parameters, PARAMETERS_DATA_LEN);

  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];

  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    //Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;
  }

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    this->errorState = readBuffer[5];
    return -1;
  }

  // Success
  return 1;
}

// Release the servo (i.e. turn off its motor)
void servoST3215_releaseServo(ServoST3215 *this) {
  if (!writeServoReg8(this->servoId, SMS_STS_TORQUE_ENABLE, 0x00)) {
    //Serial.println("ERROR RELEASING SERVO");
    this->errorState = 1;
  }
}

// Torque the servo motor (i.e. turn on the servo motor, holding its current position)
void servoST3215_torqueServo(ServoST3215 *this) {
  if (!writeServoReg8(this->servoId, SMS_STS_TORQUE_ENABLE, 0x01)) {
    //Serial.println("ERROR TORQUING SERVO");
    this->errorState = 1;
  }
}

// Stop the servo motor.
// This is accomplished by releasing it, then quickly torquing it to hold in the current position
void servoST3215_stopServo(ServoST3215 *this) {
  servoST3215_releaseServo(this);
  //delay(2);
  servoST3215_torqueServo(this);
}

