/*
 * st3215.h
 *
 *  Created on: Jan 26, 2024
 *      Author: alex
 */

#include <stdbool.h>

#ifndef MODULES_ST3215_ST3215_H_
#define MODULES_ST3215_ST3215_H_

// Register map (from the Waveshare driver)
// EPROM
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

// EPROM
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

// SRAM
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

// SRAM
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70


typedef struct {

    // Accessors (updated by poll() )
    uint8_t   servoId;
    uint16_t  load;
    uint16_t  currentSpeed;
    uint8_t  voltage;
    uint16_t  current;
    uint16_t  currentPosition;
    uint8_t  mode;
    uint8_t  temp;
    uint8_t  isMoving;

    uint8_t   doesExist;      // Does this servo exist (i.e. respond to ping?)
    uint8_t   errorState;     // Current error state
} ServoST3215;


extern void servoST3215_init(ServoST3215 *this, uint8_t id, void *dHUART);
// Does the servo exist (i.e. respond to ping?)
extern bool servoST3215_exists(ServoST3215 *item);
// Ping the servo
extern int servoST3215_ping(ServoST3215 *item);
// Poll data from servo memory
extern void servoST3215_poll(ServoST3215 *item);

// Motor motion commands
extern int servoST3215_movePosition(ServoST3215 *item, uint16_t newPosition, uint16_t speed, uint8_t acceleration);
extern void servoST3215_releaseServo(ServoST3215 *item);
extern void servoST3215_torqueServo(ServoST3215 *item);
extern void servoST3215_stopServo(ServoST3215 *item);


#endif /* MODULES_ST3215_ST3215_H_ */
