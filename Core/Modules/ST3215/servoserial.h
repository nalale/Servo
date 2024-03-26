/*
 * SerialProtocol.h
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#ifndef MODULES_ST3215_SERVOSERIAL_H_
#define MODULES_ST3215_SERVOSERIAL_H_

// Waveshare board: The UART used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// Instructions
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83
#define INST_RECOVERY 0x06

typedef enum
{
    SERVO_READY            = 1<<0,         /*!< Startup finished. */
	SERVO_PACKET_RECEIVED   = 1<<1,         /*!< Frame received. */
	SERVO_CMD_EXECUTE          = 1<<2,         /*!< Execute function. */
	SERVO_PACKET_SENT       = 1<<3          /*!< Frame sent. */
} ServoEventType_t;

// Function prototypes
void ServoSerialBusInit();
void ServoSerialBusStart( void );
void ServoSerialBusStop( void );

bool ServoSerialEventGet(ServoEventType_t *Event);
bool ServoSerialTimeoutCheck(void);
void ServoSerialWaitingExpired(void);
void ServoSerialPacketProcessed(void);

bool ServoSerialStatusGet(uint8_t *Status);
bool ServoSerialPacketGet(uint8_t *Len, uint8_t *Data[]);
bool ServoSerialIdGet(uint8_t *ServoID);

int readServoMemory(uint8_t servoId, uint8_t startAddress, uint8_t length);
int writeServoReg8(uint8_t servoId, uint8_t regAddress, uint8_t data);

uint16_t parse16BitUInt(uint8_t highByte, uint8_t lowByte);
void packU16toButter(uint16_t byteToPack, uint8_t* buffer);
int16_t parse16BitInt(uint8_t highByte, uint8_t lowByte);

void sendServoPacket(uint8_t servoID, uint8_t instruction, uint8_t* parameters, uint8_t length);
int readServoPacket(uint8_t* readBuffer, uint8_t length);
uint8_t receivePacket(void);

void sendAtomPacket(uint8_t instruction, uint8_t* parameters, uint8_t length);

#endif /* MODULES_ST3215_SERVOSERIAL_H_ */
