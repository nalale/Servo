/*
 * servo_control.h
 *
 *  Created on: Feb 10, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef SRC_SERVO_CONTROL_H_
#define SRC_SERVO_CONTROL_H_

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"

#include "ST3215/ST3215.h"

#define DEFAULT_SERVO_ID 	1

typedef enum { No_Action = 0, SwitchAction_Stop, SwitchAction_Slowdown } SwitchAction_t;

typedef struct {
	uint16_t RegType;
	uint16_t iRegIndex;
	uint16_t InData;
} QueueCmd_t;



// указатель на функцию записи параметра
typedef int (*pWriteParam)(ServoST3215* item, int16_t param);

void ServoCtrl_Init(void *phuart);
void ServoCtrl_Process(void);

void ChangeParametersRequest(uint16_t RegType, uint16_t iRegIndex, uint16_t InData);

#endif /* SRC_SERVO_CONTROL_H_ */
