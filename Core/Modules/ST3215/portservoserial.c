/*
 * portservoserial.c
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdint.h>
#include "portservoserial.h"

#include "main.h"

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
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == uart->Instance)
	{
		pxMBFrameCBByteReceived();
		HAL_UART_Receive_IT(uart, &singlechar, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == uart->Instance)
	{
		pxMBFrameCBTransmitterEmpty();
	}
}
*/


