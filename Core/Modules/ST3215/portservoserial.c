/*
 * portservoserial.c
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdint.h>

#include "main.h"
#include "portservoserial.h"

static UART_HandleTypeDef *uart;
static uint8_t singlechar;

bool PortServoSerialInit( void *dHUART, uint32_t ulBaudRate, void *dHTIM )
{
	uart = (UART_HandleTypeDef *)dHUART;

	return true;
}

void PortServoSerialEnable(bool xRxEnable, bool xTxEnable)
{
	if(xRxEnable)
	{
		//RS485_RTS_LOW;
		HAL_UART_Receive_IT(uart, &singlechar, 1);
	}
	else
	{
		HAL_UART_AbortReceive_IT(uart);
	}

	if(xTxEnable)
	{
		//RS485_RTS_HIGH;
		//pxMBFrameCBTransmitterEmpty();
	}
	else
	{
		HAL_UART_AbortTransmit_IT(uart);
	}
}

void PortServoClose(void)
{
	HAL_UART_AbortReceive_IT(uart);
	HAL_UART_AbortTransmit_IT(uart);
}

bool PortServoSerialPutByte(int8_t ucByte)
{
	HAL_UART_Transmit_IT(uart, (uint8_t*)&ucByte, 1);
	return true;
}

bool PortServoSerialPutBytes(volatile uint8_t *ucByte, uint16_t usSize)
{
	HAL_UART_Transmit_IT(uart, (uint8_t *)ucByte, usSize);
	return true;
}

bool PortServoSerialGetByte(int8_t * pucByte)
{
	*pucByte = (uint8_t)(singlechar);
	return true;
}


uint32_t PortServoSerialGetMsNow() {
	return HAL_GetTick();
}

uint8_t PortServoSerial_RxCpltCallback(void *huart)
{
	if(((UART_HandleTypeDef*)huart)->Instance == uart->Instance)
	{
		//pxMBFrameCBByteReceived();
		HAL_UART_Receive_IT(uart, &singlechar, 1);
		return 1;
	}
	return 0;
}

uint8_t PortServoSerial_TxCpltCallback(void *huart)
{
	if(((UART_HandleTypeDef*)huart)->Instance == uart->Instance)
	{
		//pxMBFrameCBTransmitterEmpty();
		return 1;
	}
	return 0;
}



