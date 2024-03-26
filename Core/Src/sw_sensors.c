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

static uint8_t sw_neg_stop = 0, sw_neg_slow = 0, sw_pos_stop = 0, sw_pos_slow = 0;
static uint32_t normal_speed = 0;

static void ServoSwitchProc(uint8_t sens, uint8_t swState, uint8_t swPrevState);
static void MotorSwitchProc(uint8_t sens, uint8_t swState, uint8_t swPrevState);

void sw_sens_handle() {
	// захват данных
	shift_reg_latch_data();

	// транслируем входа регистров в битовый массив
	shift_reg_switches_get(&ucSDiscInBuf[MB_IDX_DIN_SW_1_8_STATE], MB_DIN_DEV_DATA_LEN);

	/*
	 * 1. Проверяем датчики, привязанные к выбранному серво
	 * 2. Если датчик активен, вызываем действие привязанное к датчику
	 * Датчики 0, 1 - отвечают за -ое направление
	 * Датчики 2, 3 - отвечают за +ое направление
	 */
	for(uint8_t sens = 0; sens < 4; sens++) {
		SwitchSensState_t swState = SwitchSens_Empty;
		MBRegsTableNote_t *note = app_mb_note_find(MB_IDX_CFG_SW_1_NUM + sens);
		uint8_t sens_num = *note->pMBRegValue;

		// Если датчик не выбран проверить следущий
		if(sens_num == 0)
			continue;

		// получить состояние датчика
		shift_reg_data_get(sens_num, &swState);
		// получить предыдущее состояние датчика
		uint8_t swPrevState = 0x1 & (ucSDiscInBuf[MB_IDX_SERVO_SW_STATES] >> sens);

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
			ucSDiscInBuf[MB_IDX_SERVO_SW_STATES] |= 1 << sens;

			MBRegsTableNote_t *notePos = app_mb_note_find(MB_IDX_RW_OPERATE_MODE);
			uint8_t act_mode = *notePos->pMBRegValue;

			// режим серво (абсолютное перемещение)
			if(act_mode == 0) {
				ServoSwitchProc(sens, swState, swPrevState);
			}
			// режим мотор (относительное перемещение)
			else if(act_mode == 3) {
				MotorSwitchProc(sens, swState, swPrevState);
			}

		} else {
			//обновить состояние датчика "неактивен" в регистре MB
			ucSDiscInBuf[MB_IDX_SERVO_SW_STATES] &= ~(1 << sens);

			/*if((sw_neg_stop || sw_pos_stop) &&
				((swState == SwitchSens_Empty || swState == SwitchSens_Off)&& swPrevState == 1))
				ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
			*/

			if((sw_neg_slow || sw_pos_slow) &&
				((swState == SwitchSens_Empty || swState == SwitchSens_Off)&& swPrevState == 1) &&
					normal_speed > 0)
				ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, normal_speed);

			sw_neg_stop = (sens < 2)? 0 : sw_neg_stop;
			sw_pos_stop = (sens >= 2)? 0 : sw_pos_stop;

			sw_neg_slow = (sens < 2)? 0 : sw_neg_slow;
			sw_pos_slow = (sens >= 2)? 0 : sw_pos_slow;
		}
	}
}


void ServoSwitchProc(uint8_t sens, uint8_t swState, uint8_t swPrevState) {
	// какое действие нужно выполнить
	MBRegsTableNote_t *note = app_mb_note_find(MB_IDX_CFG_SW_1_ACT + sens);
	SwitchAction_t act = *note->pMBRegValue;

	note = app_mb_inreg_note_find(MB_IDX_ACTUAL_POS);
	int16_t act_pos = *note->pMBRegValue;

	switch(act) {
	case SwitchAction_Stop:

		sw_neg_stop = (sens < 2);
		sw_pos_stop = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// остановить
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TARGET_POS, act_pos);
		}
		break;
	case SwitchAction_Slowdown:
		sw_neg_slow = (sens < 2);
		sw_pos_slow = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// запомнить скорость по умолчанию
			note = app_mb_note_find(MB_IDX_RW_RUNNING_SPEED);
			normal_speed = *note->pMBRegValue;

			// добавить в очередь команду задания ускорения
			note = app_mb_note_find(MB_IDX_CFG_SLOWDOWN_DEACC);
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_ACC, *note->pMBRegValue);
			// добавить в очередь команду задани скорости
			note = app_mb_note_find(MB_IDX_CFG_SLOWDOWN_SPEED);
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, *note->pMBRegValue);
		}

		break;
	default:
		sw_neg_stop = (sens < 2)? 0 : sw_neg_stop;
		sw_pos_stop = (sens >= 2)? 0 : sw_pos_stop;

		sw_neg_slow = (sens < 2)? 0 : sw_neg_slow;
		sw_pos_slow = (sens >= 2)? 0 : sw_pos_slow;
		break;
	}

}

void MotorSwitchProc(uint8_t sens, uint8_t swState, uint8_t swPrevState) {
	// какое действие нужно выполнить
	MBRegsTableNote_t *note = app_mb_note_find(MB_IDX_CFG_SW_1_ACT + sens);
	SwitchAction_t act = *note->pMBRegValue;

	note = app_mb_inreg_note_find(MB_IDX_ACTUAL_POS);
	int16_t act_pos = *note->pMBRegValue;

	switch(act) {
	case SwitchAction_Stop:

		sw_neg_stop = (sens < 2);
		sw_pos_stop = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// остановить

			// останавливаем привод, записывая текущее положение с обратным знаком.
			// т.к записанную последнюю команду положения привод принимает за 0 отсчета.
			// т.е остановиться в данный момент, можно записав текущее положение с минусом от 0.
			note = app_mb_note_find(MB_IDX_RW_RUNNING_SPEED);
			int16_t speed = *note->pMBRegValue;
			if(act_pos < 0)
				speed = -speed;

			int16_t delta_step = 0.2f * speed;
			act_pos -= delta_step;

			//ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 0);
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TARGET_POS, -act_pos);
		}
		break;
	case SwitchAction_Slowdown:
		sw_neg_slow = (sens < 2);
		sw_pos_slow = (sens >= 2);

		if(swState == 1 && swPrevState == 0) {
			// запомнить скорость по умолчанию
			note = app_mb_note_find(MB_IDX_RW_RUNNING_SPEED);
			normal_speed = *note->pMBRegValue;

			// добавить в очередь команду задания ускорения
			note = app_mb_note_find(MB_IDX_CFG_SLOWDOWN_DEACC);
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_ACC, *note->pMBRegValue);
			// добавить в очередь команду задани скорости
			note = app_mb_note_find(MB_IDX_CFG_SLOWDOWN_SPEED);
			ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, *note->pMBRegValue);
		}

		break;
	default:
		sw_neg_stop = (sens < 2)? 0 : sw_neg_stop;
		sw_pos_stop = (sens >= 2)? 0 : sw_pos_stop;

		sw_neg_slow = (sens < 2)? 0 : sw_neg_slow;
		sw_pos_slow = (sens >= 2)? 0 : sw_pos_slow;
		break;
	}
}

uint8_t sw_sens_check_condition(uint16_t InData) {
	// Если активен левый концевик и направление отрицательное
	if(sw_neg_stop) {
		if((int16_t)InData <= 0)
			return ERROR;
		//else
		//	ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
	}


	// Если активен правый концевик и направлени положительное
	if(sw_pos_stop) {
		if((int16_t)InData > 0)
			return ERROR;
		//else
		//	ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_TORQUE_LIM, 1000);
	}

	if(sw_neg_slow && ((int16_t)InData > 0)) {
		ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, normal_speed);
	}

	if(sw_pos_slow && ((int16_t)InData < 0)) {
		ChangeParametersRequest(MB_RegType_HoldReg, MB_IDX_RW_RUNNING_SPEED, normal_speed);
	}

	return SUCCESS;
}
