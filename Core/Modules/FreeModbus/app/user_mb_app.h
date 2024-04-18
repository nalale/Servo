#ifndef _USER_MB_APP_
#define _USER_MB_APP_

#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbutils.h"

/* -----------------------Slave Defines -------------------------------------*/

#define MB_IDX_SERVO_DATA_START		64

typedef enum {
	MB_INREG_DEV_FW_VER_MAJ = 0,
	MB_INREG_DEV_FW_VER_MIN,
	MB_INREG_DEV_HW_VER_MAJ,
	MB_INREG_DEV_HW_VER_MIN,
	MB_INREG_DEV_SERVO_NUM,

	MB_INDEG_DEV_DATA_LEN,		// определяет количество регистров для устройства, а не привода

	MB_IDX_FW_MAIN_VER = MB_IDX_SERVO_DATA_START,
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

#define MB_INREGS_DEV_CNT	(MB_IDX_RO_CNT - MB_IDX_SERVO_DATA_START)
#define MB_INREGS_CNT		((MB_IDX_RO_CNT - MB_IDX_SERVO_DATA_START) + MB_INDEG_DEV_DATA_LEN)

typedef enum {
	MB_IDX_CFG_ADDRESS = 0,
	MB_IDX_CFG_MD_BD,			// скорость modbus
	MB_IDX_CFG_SERVO_BD,		// скорость обмена с серво

	MB_IDX_DEV_CFG_DATA_LEN,

	// для группового управления
	MB_IDX_RW_GROUP_1_ID = 10,
	MB_IDX_RW_GROUP_2_ID,
	MB_IDX_RW_GROUP_3_ID,
	MB_IDX_RW_GROUP_1_POS,
	MB_IDX_RW_GROUP_2_POS,
	MB_IDX_RW_GROUP_3_POS,
	MB_IDX_RW_GROUP_1_SPEED,
	MB_IDX_RW_GROUP_2_SPEED,
	MB_IDX_RW_GROUP_3_SPEED,

	MB_IDX_RW_UNIT_DATA_LEN,		// определяет количество регистров для устройства, а не привода


	// для индивидуального управления
	MB_IDX_RW_SERVO_ID = MB_IDX_SERVO_DATA_START,
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

	MB_IDX_CFG_SW_1_NUM,		// датчик при вращении в - сторону
	MB_IDX_CFG_SW_2_NUM,		// датчик при вращении в - сторону
	MB_IDX_CFG_SW_3_NUM,		// датчик при вращении в + сторону
	MB_IDX_CFG_SW_4_NUM,		// датчик при вращении в + сторону
	MB_IDX_CFG_SW_1_ACT,
	MB_IDX_CFG_SW_2_ACT,
	MB_IDX_CFG_SW_3_ACT,
	MB_IDX_CFG_SW_4_ACT,
	MB_IDX_CFG_SLOWDOWN_DEACC,
	MB_IDX_CFG_SLOWDOWN_SPEED,

	MB_SERVO_DATA_LEN,

}MB_HoldRegs_Servo_t;

#define MB_HOLDREGS_CNT	((MB_SERVO_DATA_LEN - MB_IDX_SERVO_DATA_START) + (MB_IDX_RW_UNIT_DATA_LEN - MB_IDX_RW_GROUP_1_ID) + MB_IDX_DEV_CFG_DATA_LEN)

typedef enum {
	MB_IDX_COIL_FLASH_WRITE = 0,
	MB_COIL_DEV_DATA_LEN,

	MB_IDX_COIL_RESP_STATUS = MB_IDX_SERVO_DATA_START,
	MB_IDX_COIL_LOCK_MARK,

	MB_COILS_DATA_LEN,
} MB_Coils_Servo_t;

#define MB_COILS_CNT	((MB_COILS_DATA_LEN - MB_IDX_SERVO_DATA_START) + MB_COIL_DEV_DATA_LEN)

typedef enum {
	MB_IDX_DIN_SW_1_8_STATE = 0,
	MB_IDX_DIN_SW_9_16_STATE,
	MB_IDX_DIN_SW_17_24_STATE,
	MB_IDX_DIN_SW_25_32_STATE,
	MB_IDX_DIN_SW_33_40_STATE,
	MB_IDX_DIN_SW_41_48_STATE,
	MB_IDX_DIN_SW_49_50_STATE,
	MB_DIN_DEV_DATA_LEN,

	MB_IDX_ACTUAL_SERVO_STATES = MB_IDX_SERVO_DATA_START,				// online flag, moving flag, sw1, sw2, sw3, sw4

	MB_DIN_DATA_LEN,
} MB_DIns_Servo_t;

#define MB_DIN_CNT	((MB_DIN_DATA_LEN - MB_IDX_SERVO_DATA_START) + MB_DIN_DEV_DATA_LEN)

#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   64 //18 * 8

#define S_COIL_START                  0
#define S_COIL_NCOILS                 66 // MB_COILS_DATA_LEN//64

#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             75

#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           108 //MB_SERVO_DATA_LEN //100

/* salve mode: holding register's all address */
#define          S_HD_RESERVE                     0
#define          S_HD_CPU_USAGE_MAJOR             1
#define          S_HD_CPU_USAGE_MINOR             2
/* salve mode: input register's all address */
#define          S_IN_RESERVE                     0
/* salve mode: coil's all address */
#define          S_CO_RESERVE                     0
/* salve mode: discrete's all address */
#define          S_DI_RESERVE                     0

extern void (*eMBRegHoldingWriteCB)(USHORT RegType, USHORT iRegIndex, USHORT usNRegs);
extern int8_t (*eMBRegHoldingReadCB)(USHORT RegType, USHORT iRegIndex, USHORT *uspData);

extern int8_t (*eMBRegInputReadCB)(USHORT RegType, USHORT iRegIndex, USHORT *uspData);

#endif // _USER_MB_APP_
