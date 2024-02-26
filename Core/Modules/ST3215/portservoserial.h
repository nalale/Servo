/*
 * portservoserial.h
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#ifndef MODULES_ST3215_PORTSERVOSERIAL_H_
#define MODULES_ST3215_PORTSERVOSERIAL_H_

#include <stdbool.h>

bool PortServoSerialInit( void *dHUART, uint32_t ulBaudRate, void *dHTIM );
void PortServoSerialEnable(bool xRxEnable, bool xTxEnable);
void PortServoClose(void);
bool PortServoSerialPutByte(int8_t ucByte);
bool PortServoSerialPutBytes(volatile uint8_t *ucByte, uint16_t usSize);
bool PortServoSerialGetByte(int8_t * pucByte);

#endif /* MODULES_ST3215_PORTSERVOSERIAL_H_ */
