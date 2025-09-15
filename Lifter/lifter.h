/*
 * lifter.h
 *
 *  Created on: Aug 4, 2025
 *      Author: 88698
 */

#ifndef INC_LIFTER_H_
#define INC_LIFTER_H_

// Include C++ headers outside extern "C" block
#ifdef __cplusplus
#include "../Drivers/VL53L0X_Class/VL53L0X_Class.h"

// C++ variable declarations (must be outside extern "C")
extern VL53L0X_Enhanced front_sensor;
extern VL53L0X_Enhanced back_sensor;
extern VL53L0X_Enhanced sensors[2];
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "DC_motor.h"

// Declare external TIM handles
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim23;

// Declare lifter motors
extern DC_motor lifter_right_wheel;
extern DC_motor lifter_left_wheel;
extern DC_motor front_lifter;
extern DC_motor back_lifter;

// Declare state variables
extern DC_motor lifters[2];
extern int directions[2];
extern int moveStates[2];
extern int goalHeights[2];
extern float wheel_sp;

// Debug status variables (monitor these in live expressions)
extern int back_sensor_status;   // 0=not started, 1=initializing, 2=success, -1=failed
extern int front_sensor_status;  // 0=not started, 1=initializing, 2=success, -1=failed
extern int i2c_test_back;        // 0=not tested, 1=ok, -1=failed
extern int i2c_test_front;       // 0=not tested, 1=ok, -1=failed
extern int setup_step;           // Track which step we're on
extern int vl53_init_error;      // Store VL53L0X initialization error code
extern int gpio_test_result;     // Test GPIO pin state
extern int i2c_bus_test;         // -1=pullup issue, -2=config issue, 1=ok

// Distance variables for live monitoring
extern uint16_t back_lifter_distance;    // Current back sensor distance (mm)
extern uint16_t front_lifter_distance;   // Current front sensor distance (mm)
extern bool back_sensor_valid;           // Back sensor measurement validity
extern bool front_sensor_valid;          // Front sensor measurement validity
//extern bool achieve[2];

// Function declarations
void lifterSetup();
void measureDistance();
void testSensors();
void testBothSensors();  // New function to test both sensors
void scanI2CBus();       // New function to scan I2C bus
void lifterMove(int lifterIndex, bool direction, bool move);
void lifterRun();
void setHeight(int lifterIndex, int height);

}

#ifdef __cplusplus
// Enhanced VL53L0X sensor functions (C++ only)
uint16_t getLifterPosition(int lifter);
bool areLifterSensorsReady();
bool calibrateLifterSensors(uint16_t target_distance_mm);
void printLifterStatus();
#endif

#endif /* INC_LIFTER_H_ */
