/*
 * sw_sensors.c
 *
 *  Created on: Mar 25, 2024
 *      Author: SoftwareEngineer_01
 */
#include "mb_app_data.h"
#include "servo_control.h"
#include "74HC165.h"

extern uint8_t ucSDiscInBuf[];

static void ServoSwitchProc(ServoST3215 *servo, uint8_t sens, uint8_t swState, uint8_t swPrevState);
static void MotorSwitchProc(ServoST3215 *servo, uint8_t sens, uint8_t swState, uint8_t swPrevState);

void sw_sens_handle(ServoST3215 *servo) {

	/*
	 * 1. Проверяем датчики, привязанные к выбранному серво
	 * 2. Если датчик активен, вызываем действие привязанное к датчику
	 * Датчики 0, 1 - отвечают за -ое направление
	 * Датчики 2, 3 - отвечают за +ое направление
	 */
	for(uint8_t sens = 0; sens < 4; sens++) {
		SwitchSensState_t swState = SwitchSens_Empty;

		// арифметика указателей
		uint8_t sens_num = *((uint16_t*)&servo->RW_MemData.sw_sens_cfg.CFG_SW_1_NUM + sens);
		// Если датчик не выбран проверить следущий
		if(sens_num == 0)
			continue;

		// получить состояние датчика
		shift_reg_data_get(sens_num, &swState);
		// получить предыдущее состояние датчика
		uint8_t swPrevState = 0x1 & ((servo->RO_MemData.dins_data.DiscreteStates >> 2) >> sens);

		// обработка датчика
		if(swState == SwitchSens_On) {
			/*
			 * 1. Режим мотор (3).
			 * Если активен датчик 0, 1 - запретить -ое направление
			 * Если активен датчик 2, 3 - запретить +ое направление
			 *
			 * 2. Режим серво (0).
			 * Если сработал датчик, запомнить Actual Pos.
			 * Если следующая посылка TargetPos меньше ActualPos при + знаке, то вращаем, иначе игнорировать.
			 * Если следующая посылка TargetPos больше ActualPos при - знаке, то вращаем, иначе игнорировать.
			 */

			//обновить состояние датчика "активен" в регистре MB
			servo->RO_MemData.dins_data.DiscreteStates |= 1 << (sens + 2);

			//MBRegsTableNote_t *notePos = app_mb_note_find(MB_IDX_RW_OPERATE_MODE);
			//uint8_t act_mode = *notePos->pMBRegValue;

			uint16_t act_mode = servo->RW_MemData.eeprom_data.OpMode;

			// режим серво (абсолютное перемещение)
			if(act_mode == 0) {
				ServoSwitchProc(servo, sens, swState, swPrevState);
			}
			// режим мотор (относительное перемещение)
			else if(act_mode == 3) {
				MotorSwitchProc(servo, sens, swState, swPrevState);
			}

		} else {
			//обновить состояние датчика "неактивен" в регистре MB
			servo->RO_MemData.dins_data.DiscreteStates &= ~(1 << (sens + 2));

			/*if((sw_neg_stop || sw_pos_stop) &&
				((swState == SwitchSens_Empty || swState == SwitchSens_Off)&& swPrevState == 1))
				ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
			*/

			if((servo->sw_neg_slow || servo->sw_pos_slow) &&
				((swState == SwitchSens_Empty || swState == SwitchSens_Off)&& swPrevState == 1) &&
				servo->normal_speed > 0)
				eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, servo->normal_speed);

			servo->sw_neg_stop = (sens < 2)? 0 : servo->sw_neg_stop;
			servo->sw_pos_stop = (sens >= 2)? 0 : servo->sw_pos_stop;

			servo->sw_neg_slow = (sens < 2)? 0 : servo->sw_neg_slow;
			servo->sw_pos_slow = (sens >= 2)? 0 : servo->sw_pos_slow;
		}
	}
}


void ServoSwitchProc(ServoST3215 *servo, uint8_t sens, uint8_t swState, uint8_t swPrevState) {
	// какое действие нужно выполнить
	SwitchAction_t act = *((uint16_t*)&servo->RW_MemData.sw_sens_cfg.CFG_SW_1_ACT + sens);

	int16_t act_pos = servo->RO_MemData.flash_data.actual_pos;

	switch(act) {
	case SwitchAction_Stop:

		servo->sw_neg_stop = (sens < 2);
		servo->sw_pos_stop = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// остановить
			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_TARGET_POS, act_pos);
		}
		break;
	case SwitchAction_Slowdown:
		servo->sw_neg_slow = (sens < 2);
		servo->sw_pos_slow = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// запомнить скорость по умолчанию
			servo->normal_speed = servo->RW_MemData.flash_data.cmdSpeed;

			// добавить в очередь команду задания ускорения
			int16_t acc = servo->RW_MemData.sw_sens_cfg.CFG_SLOWDOWN_DEACC;
			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_ACC, acc);

			// добавить в очередь команду задани скорости
			int16_t speed = servo->RW_MemData.sw_sens_cfg.CFG_SLOWDOWN_SPEED;
			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, speed);
		}

		break;
	default:
		servo->sw_neg_stop = (sens < 2)? 0 : servo->sw_neg_stop;
		servo->sw_pos_stop = (sens >= 2)? 0 : servo->sw_pos_stop;

		servo->sw_neg_slow = (sens < 2)? 0 : servo->sw_neg_slow;
		servo->sw_pos_slow = (sens >= 2)? 0 : servo->sw_pos_slow;
		break;
	}

}

void MotorSwitchProc(ServoST3215 *servo, uint8_t sens, uint8_t swState, uint8_t swPrevState) {
	// какое действие нужно выполнить
	SwitchAction_t act = *((uint16_t*)&servo->RW_MemData.sw_sens_cfg.CFG_SW_1_ACT + sens);

	int16_t act_pos = servo->RO_MemData.flash_data.actual_pos;

	switch(act) {
	case SwitchAction_Stop:

		servo->sw_neg_stop = (sens < 2);
		servo->sw_pos_stop = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// остановить

			// останавливаем привод, записывая текущее положение с обратным знаком.
			// т.к записанную последнюю команду положения привод принимает за 0 отсчета.
			// т.е остановиться в данный момент, можно записав текущее положение с минусом от 0.
			int16_t speed = servo->RW_MemData.flash_data.cmdSpeed;
			if(act_pos < 0)
				speed = -speed;

			int16_t delta_step = 0.2f * speed;
			act_pos -= delta_step;

			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_TARGET_POS, -act_pos);
		}
		break;
	case SwitchAction_Slowdown:
		servo->sw_neg_slow = (sens < 2);
		servo->sw_pos_slow = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// запомнить скорость по умолчанию
			servo->normal_speed = servo->RW_MemData.flash_data.cmdSpeed;

			// добавить в очередь команду задания ускорения
			int16_t acc = servo->RW_MemData.sw_sens_cfg.CFG_SLOWDOWN_DEACC;
			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_ACC, acc);
			// добавить в очередь команду задани скорости
			int16_t speed = servo->RW_MemData.sw_sens_cfg.CFG_SLOWDOWN_SPEED;
			eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, speed);
		}

		break;
	default:
		servo->sw_neg_stop = (sens < 2)? 0 : servo->sw_neg_stop;
		servo->sw_pos_stop = (sens >= 2)? 0 : servo->sw_pos_stop;

		servo->sw_neg_slow = (sens < 2)? 0 : servo->sw_neg_slow;
		servo->sw_pos_slow = (sens >= 2)? 0 : servo->sw_pos_slow;
		break;
	}
}

uint8_t sw_sens_check_condition(ServoST3215 *servo, uint16_t InData) {
	// Если активен левый концевик и направление отрицательное
	if(servo->sw_neg_stop) {
		if((int16_t)InData <= 0)
			return ERROR;
		//else
		//	ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
	}

	// Если активен правый концевик и направлени положительное
	if(servo->sw_pos_stop) {
		if((int16_t)InData > 0)
			return ERROR;
		//else
		//	ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
	}

	if(servo->sw_neg_slow && ((int16_t)InData > 0)) {
		eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, servo->normal_speed);
	}

	if(servo->sw_pos_slow && ((int16_t)InData < 0)) {
		eMBRegHoldingWriteCB(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, servo->normal_speed);
	}

	return SUCCESS;
}
