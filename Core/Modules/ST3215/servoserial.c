/*
 * SerialProtocol.c
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */



#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "servoserial.h"
#include "portservoserial.h"

#define SERIAL_FRAME_LEN	15

//HardwareSerial *pSerial;
static void *UART;
static unsigned long startTime = 0;
static ServoEventType_t ServoSerialEvent = 0;
static uint16_t ReadAddres = 0;
static uint16_t ReadLen = 0;
static uint16_t servoId = 0;

// Low-level function to wait for a read packet from a servo
#define READ_TIMEOUT_MILLIS     50
#define SERVO_PACKET_HEADER_CHAR  0xFF
#define SERVO_PACKET_DATA_LEN_POS	3


static int numBytesRecieved = 0;
static int numFFs = 0;
static uint8_t bytes_await = 0;

void ServoSerialBusInit(void *dHUART) {
  //Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  UART = dHUART;
  ServoSerialEvent = SERVO_READY;
  PortServoSerialInit(UART, 1000000, NULL);
}

void ServoSerialBusStart( void )
{
    //ENTER_CRITICAL_SECTION(  );
    PortServoSerialEnable( true, false );
    //vMBPortTimersEnable(  );

    //EXIT_CRITICAL_SECTION(  );
}

void ServoSerialBusStop( void )
{
    //ENTER_CRITICAL_SECTION(  );
    PortServoSerialEnable( false, false );
    //vMBPortTimersDisable(  );
    //EXIT_CRITICAL_SECTION(  );
}

int reqServoCmd(uint8_t ServoId, uint8_t Instruction, uint8_t bytesToWrite) {
	  // header + id + len + status + crc
	  bytes_await = 6;

	  uint8_t parameters[bytesToWrite];

	  sendServoPacket(ServoId, Instruction, (uint8_t*)parameters, bytesToWrite);

	  return 1;
}

// Read a block of memory from a given servo.
//   servoID:         The ID of the servo to read from.
//   startAddress:    The start address to begin reading from
//   numBytesToRead:  The number of bytes to read, starting from the start address
// NOTE THAT READBUFFER SHOULD ALWAYS INCLUDE THE 6 EXTRA BYTES FOR THE HEADER.
int readServoMemory(uint8_t ServoId, uint8_t startAddress, uint8_t numBytesToRead) {

	ReadAddres = startAddress;
	servoId = ServoId;

	if(ReadLen == 0)
		ReadLen = numBytesToRead;

	if(numBytesToRead > SERIAL_FRAME_LEN)
		numBytesToRead = SERIAL_FRAME_LEN;

	bytes_await = numBytesToRead + 6;

  // First, send the read request
  uint8_t parameters[] = {startAddress, numBytesToRead};
  sendServoPacket(ServoId, INST_READ, (uint8_t*)parameters, 2);

  // Then, read the response
/*  int totalBytesToRead = numBytesToRead + 6;    // 5 in the header, +1 checksum
  int totalBytesRead = readServoPacket(readBuffer, numBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;
  }
*/
  return 1;
}

int writeServoReg8(uint8_t servoId, uint8_t regAddress, uint8_t data) {
  // First, send the read request
  uint8_t parameters[] = {regAddress, data};
  sendServoPacket(servoId, INST_WRITE, (uint8_t*)parameters, 2);

  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];
  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;
  }

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    //Serial.printf("SERVO RETURNED NON-ZERO WORKING STATUS (%02X)", readBuffer[5]);
    return -1;
  }

  // Success
  return 1;
}

int16_t parse16BitInt(uint8_t highByte, uint8_t lowByte) {
	int16_t res = (((uint16_t)highByte) << 8) + lowByte;

	if(res < 0){
		res &= 0x7fff;
		res = -res;
	}

	return res;
}

// Convert two bytes into a 16-bit integer
uint16_t parse16BitUInt(uint8_t highByte, uint8_t lowByte) {
  return (((uint16_t)highByte) << 8) + lowByte;
}

// Pack an unsigned 16 bit integer into an 8-bit array, low byte first.
void packU16toButter(uint16_t byteToPack, uint8_t* buffer) {
  // Low byte first
  uint8_t lowByte = (uint8_t)(0x00FF & byteToPack);
  uint8_t highByte = (uint8_t)((0xFF00 & byteToPack) >> 8);

  buffer[0] = lowByte;
  buffer[1] = highByte;
}

#define MAX_WRITE_BYTES 32
#define MAX_READ_BYTES 64

static uint8_t bufferOut[MAX_WRITE_BYTES];
static uint8_t readBuffer[MAX_READ_BYTES];

// Low-level function to send a packet to a given servo
void sendServoPacket(uint8_t servoID, uint8_t instruction, uint8_t* parameters, uint8_t length) {
  // Clear buffer
  for (int i=0; i<MAX_WRITE_BYTES; i++) {
    bufferOut[i] = 0;
  }

  // Step 1: Pack the message into a transmit buffer
  // Header
  bufferOut[0] = 0xff;
  bufferOut[1] = 0xff;
  bufferOut[2] = servoID;
  bufferOut[3] = length + 2;    // +2 because the length includes the parameters (=length), plus the instruction byte (+1), plus the checksum (+1)
  bufferOut[4] = instruction;

  // Parameters
  for (int i=0; i<length; i++) {
    bufferOut[5+i] = parameters[i];
  }

  // Checksum
  int transmitLength = length + 6;  // Includes the full header
  // Note: Checksum calculation does not include the first two bytes (0xFF 0xFF)
  uint8_t checksum = 0;
  for (int i=2; i<transmitLength; i++) {
    checksum += bufferOut[i];
  }
  bufferOut[5+length] = ~checksum;  // Add the checksum byte

  ServoSerialEvent = SERVO_CMD_EXECUTE;
  startTime = PortServoSerialGetMsNow();       // Keep track of the time, in case there's a read timeout

  // Step 2: Transmit the message
  PortServoSerialPutBytes((uint8_t *)bufferOut, transmitLength);
}


bool ServoSerialEventGet(ServoEventType_t *Event) {
	*Event = ServoSerialEvent;
	return true;
}

void ServoSerialPacketProcessed(void) {
	ServoSerialEvent = SERVO_READY;

	numFFs = 0;
	numBytesRecieved = 0;
	ReadLen = 0;
	bytes_await = 0;
}

void ServoSerialWaitingExpired(void) {
	if(ServoSerialEvent == SERVO_CMD_EXECUTE)
		ServoSerialEvent = SERVO_READY;

	numFFs = 0;
	numBytesRecieved = 0;
	ReadLen = 0;
	bytes_await = 0;

	PortServoClose();
}

bool ServoSerialTimeoutCheck(void) {
	// Stop condition 2: Check to see if we've timed out
	if (PortServoSerialDiffFrom(startTime) > READ_TIMEOUT_MILLIS && ServoSerialEvent == SERVO_CMD_EXECUTE) {
	  return true;
	}
	return false;
}

uint8_t receivePacketLen() {
	return bytes_await;
}

uint8_t receivePacket(void) {
	//uint8_t bytes_await = 0;

	// Header (0xFF 0xFF)
/*	if (numFFs != 2) {
		PortServoSerialGetBytes((int8_t*)&readBuffer[0], 5);

		if((readBuffer[0] == readBuffer[1]) && (readBuffer[0] == SERVO_PACKET_HEADER_CHAR)){
			// wait other data
			// without status byte
			bytes_await = readBuffer[3] - 1;

			numFFs = 2;
			numBytesRecieved += 5;
		} else
			bytes_await = 0;
	} else */{

/*		PortServoSerialGetBytes((int8_t*)&readBuffer[numBytesRecieved], bytes_await);

		numBytesRecieved += bytes_await;
		bytes_await = (readBuffer[3] - 1) - (numBytesRecieved - 5);

		if(bytes_await <= 0) {
			bytes_await = 0;
			numBytesRecieved = 0;
			ServoSerialEvent = SERVO_PACKET_RECEIVED;
		} else if(bytes_await > SERIAL_FRAME_LEN)
			bytes_await = SERIAL_FRAME_LEN;
*/

		//uint8_t header[5];
		PortServoSerialGetBytes((int8_t*)readBuffer, 5, 0);

		if((readBuffer[0] == readBuffer[1]) && (readBuffer[0] == SERVO_PACKET_HEADER_CHAR)){
			// wait other data
			// without status byte
			bytes_await = readBuffer[3] - 1;

			numFFs = 2;
			numBytesRecieved += 5;
		}
		else
			return 0;

		// Заполняем тело пакета
		// Store the byte
		if(numBytesRecieved > 5)
			numBytesRecieved -= 5;
		// without status byte and crc
		uint8_t len_without_crc = readBuffer[3] - 2;

		PortServoSerialGetBytes((int8_t*)&readBuffer[numBytesRecieved], len_without_crc, 5);
		numBytesRecieved += len_without_crc;
		numFFs = 0;

		//ServoSerialEvent = SERVO_PACKET_RECEIVED;
		bytes_await = 0;

		// сколько байт осталось считать
		int8_t left = ReadLen - (ReadAddres + SERIAL_FRAME_LEN);

		if(left < SERIAL_FRAME_LEN && left > 0) {
			ReadAddres += SERIAL_FRAME_LEN;
			readServoMemory(servoId, ReadAddres, left);
		}
		else if(left >= SERIAL_FRAME_LEN) {
			ReadAddres += SERIAL_FRAME_LEN;
			readServoMemory(servoId, ReadAddres, SERIAL_FRAME_LEN);
		}
		else {
			ReadAddres = 0;
			readBuffer[3] =	ReadLen;
			numBytesRecieved = 0;
			ReadLen = 0;
			ServoSerialEvent = SERVO_PACKET_RECEIVED;

		}
	}

  // Return the number of bytes that were successfully read
  return bytes_await;//false; //numBytesRecieved;
}

bool ServoSerialPacketGet(uint8_t *Len, uint8_t **Data) {

	*Len = readBuffer[3];
	*Data = &readBuffer[5];
	return true;
}

bool ServoSerialIdGet(uint8_t *ServoID) {
	*ServoID = readBuffer[2];
	return true;
}

bool ServoSerialStatusGet(uint8_t *Status) {
	*Status = readBuffer[4];
	return true;
}

/*
 * ATOM
 */

// Low-level function to send a packet to a given servo
void sendAtomPacket(uint8_t instruction, uint8_t* parameters, uint8_t length) {
  uint8_t bufferOut[MAX_WRITE_BYTES];
  // Clear buffer
  for (int i=0; i<MAX_WRITE_BYTES; i++) {
    bufferOut[i] = 0;
  }

  // Step 1: Pack the message into a transmit buffer
  // Header
  bufferOut[0] = 0xfe;
  bufferOut[1] = 0xfe;
  bufferOut[2] = length + 2;    // +2 because the length includes the parameters (=length), plus the instruction byte (+1), plus the checksum (+1)
  bufferOut[3] = instruction;

  // Parameters
  for (int i=0; i<length; i++) {
    bufferOut[4+i] = parameters[i];
  }

  // Checksum
  int transmitLength = length + 5;  // Includes the full header
  /*
  // Note: Checksum calculation does not include the first two bytes (0xFF 0xFF)
  uint8_t checksum = 0;
  for (int i=2; i<transmitLength; i++) {
    checksum += bufferOut[i];
  }
  bufferOut[4+length] = ~checksum;  // Add the checksum byte
  */

  bufferOut[4+length] = 0xFA;  // Add static 0xFA to the end of packets

  // Step 2: Transmit the message
  PortServoSerialPutBytes((uint8_t *)bufferOut, transmitLength);
}

