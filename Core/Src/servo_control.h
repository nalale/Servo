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

typedef enum {
	MB_Type_DIn = 0,
	MB_Type_Coil,
	MB_Type_InReg,
	MB_Type_HoldReg,
}MB_Type_t;

typedef enum {
	MB_IDX_FW_MAIN_VER = 0,
	MB_IDX_FW_SUB_VER,
	MB_IDX_SERVO_MAIN_VER,
	MB_IDX_SERVO_SUB_VER,

	MB_IDX_ACTUAL_POS,
	MB_IDX_ACTUAL_SPEED,
	MB_IDX_ACTUAL_LOAD,
	MB_IDX_ACTUAL_VOLTAGE,
	MB_IDX_ACTUAL_TEMPERATURE,
	MB_IDX_ACTUAL_SERVO_STATUS,
	MB_IDX_ACTUAL_CURRENT,

	MB_IDX_RO_CNT,
} MB_InRegs_Servo_t;

typedef enum {
	MB_IDX_RW_ADDRESS = 0,
	MB_IDX_RW_MD_BD,		// скорость modbus
	MB_IDX_RW_SERVO_BD,		// скорость обмена с серво

	// для группового управления
	MB_IDX_RW_GROUP_1_ID = 10,
	MB_IDX_RW_GROUP_1_POS,
	MB_IDX_RW_GROUP_1_DUTY,
	MB_IDX_RW_GROUP_1_SPEED,
	MB_IDX_RW_GROUP_2_ID,
	MB_IDX_RW_GROUP_2_POS,
	MB_IDX_RW_GROUP_2_DUTY,
	MB_IDX_RW_GROUP_2_SPEED,
	MB_IDX_RW_GROUP_3_ID,
	MB_IDX_RW_GROUP_3_POS,
	MB_IDX_RW_GROUP_3_DUTY,
	MB_IDX_RW_GROUP_3_SPEED,

	// для индивидуального управления
	MB_IDX_RW_SERVO_ID = 128,
	MB_IDX_RW_BAUDRATE,
	MB_IDX_RW_RET_DELAY,
	MB_IDX_RW_MIN_ANGLE_LIM,
	MB_IDX_RW_MAX_ANGLE_LIM,
	MB_IDX_RW_MAX_TEMP_LIM,
	MB_IDX_RW_MAX_IN_VOLTAGE,
	MB_IDX_RW_MIN_IN_VOLTAGE,
	MB_IDX_RW_MAX_TORQUE,
	MB_IDX_RW_PHASE,
	MB_IDX_RW_UNLOAD_CONDTITION,
	MB_IDX_RW_LED_ALARM_CONDITION,
	MB_IDX_RW_P_COEF,
	MB_IDX_RW_D_COEF,
	MB_IDX_RW_I_COEF,
	MB_IDX_RW_MIN_START_FORCE,
	MB_IDX_RW_CW_INSENS_AREA,
	MB_IDX_RW_CCW_INSENS_AREA,
	MB_IDX_RW_PROT_CURRENT,
	MB_IDX_RW_ANGULAR_RESOLUTION,
	MB_IDX_RW_POS_CORRECTION,
	MB_IDX_RW_OPERATE_MODE,
	MB_IDX_RW_PROT_TORQUE,
	MB_IDX_RW_PROT_TIME,
	MB_IDX_RW_OVERLOAD_TORQUE,
	MB_IDX_RW_SPEED_P_COEF,
	MB_IDX_RW_OVERCURR_PROT_TIME,
	MB_IDX_RW_SPEED_I_COEF,

	MB_IDX_RW_TORQUE_SW,
	MB_IDX_RW_ACC,
	MB_IDX_RW_TARGET_POS,
	MB_IDX_RW_RUNNING_TIME,
	MB_IDX_RW_RUNNING_SPEED,
	MB_IDX_RW_TORQUE_LIM,

	MB_IDX_RW_CNT,

}MB_HoldRegs_Servo_t;

typedef enum {
	MB_IDX_COIL_RESP_STATUS = 0,
	MB_IDX_COIL_LOCK_MARK,
} MB_Coils_Servo_t;

typedef enum {
	MB_IDX_ACTUAL_ASYNC_WRITE_FLAG = 0,
	MB_IDX_ACTUAL_MOVING_FLAG,
	MB_IDX_ONLINE_FLAG,						// result ping instruct
} MB_DIns_Servo_t;

// указатель на функцию записи параметра
typedef int (*pWriteParam)(ServoST3215* item, int32_t param);

typedef struct {
	uint16_t MBType;
	uint16_t MBRegNum;
	pWriteParam pWriteParam;
	uint16_t ServoRegNum;
	uint16_t ParamsLen;
} MBServoSerialNode_t;

void ServoCtrl_Init(void *phuart);
void ServoCtrl_Poll(void);

#endif /* SRC_SERVO_CONTROL_H_ */
