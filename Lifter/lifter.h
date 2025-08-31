/*
 * lifter.h
 *
 *  Created on: Aug 4, 2025
 *      Author: 88698
 */

#ifndef INC_LIFTER_H_
#define INC_LIFTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "DC_motor.h"
#include "VL53.h"



// Declare external TIM handles
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim23;

// Declare lifter motors and sensors
extern DC_motor lifter_right_wheel;
extern DC_motor lifter_left_wheel;
extern DC_motor front_lifter;
extern DC_motor back_lifter;

extern VL53L0X front_sensor;
extern VL53L0X back_sensor;

// Declare state variables
extern DC_motor lifters[2];
extern VL53L0X sensors[2];
extern int directions[2];
extern int moveStates[2];
extern int goalHeights[2];
extern float wheel_sp;
//extern bool achieve[2];

// Function declarations
void lifterSetup();
void measureDistance();
void lifterMove(int lifterIndex, bool direction, bool move);
void lifterRun();
void setHeight(int lifterIndex, int height);

}

#ifdef __cplusplus

#endif

#endif /* INC_LIFTER_H_ */
