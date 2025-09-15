/*
 * lifter.cpp
 *
 *  Created on: Aug 4, 2025
 *      Author: 88698
 */

#include "lifter.h"
#include "DC_motor.h"
#include "stm32h7xx_hal.h"
#include "../Drivers/VL53L0X_Class/VL53L0X_Class.h"

// External I2C handle
extern I2C_HandleTypeDef hi2c1;

// Status variables for debugging (watch these in live expressions)
int back_sensor_status = 0;    // 0=not started, 1=initializing, 2=success, -1=failed
int front_sensor_status = 0;   // 0=not started, 1=initializing, 2=success, -1=failed
int i2c_test_back = 0;         // 0=not tested, 1=ok, -1=failed
int i2c_test_front = 0;        // 0=not tested, 1=ok, -1=failed
int setup_step = 0;            // Track which step we're on
int vl53_init_error = 0;       // Store VL53L0X initialization error code
int gpio_test_result = 0;      // Test GPIO pin state
int i2c_bus_test = 0;         // -1=pullup issue, -2=config issue, 1=ok

// Distance variables for live monitoring
uint16_t back_lifter_distance = 0;    // Current back sensor distance (mm)
uint16_t front_lifter_distance = 0;   // Current front sensor distance (mm)
bool back_sensor_valid = false;       // Back sensor measurement validity
bool front_sensor_valid = false;      // Front sensor measurement validity

// Simple I2C test function to check if sensors are reachable
bool testI2CConnection(uint8_t address) {
    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 3, 100);
    return (result == HAL_OK);
}

// Test I2C bus for common issues
int testI2CBus() {
    // Test I2C with short timeout - if this fails, likely pull-up issue
    HAL_StatusTypeDef result1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x00, 1, 10);
    
    // Test with longer timeout
    HAL_StatusTypeDef result2 = HAL_I2C_IsDeviceReady(&hi2c1, 0x00, 3, 100);
    
    if (result1 == HAL_TIMEOUT && result2 == HAL_TIMEOUT) {
        return -1; // Likely pull-up resistor issue
    } else if (result1 == HAL_ERROR || result2 == HAL_ERROR) {
        return -2; // I2C configuration issue  
    } else {
        return 1; // I2C bus OK
    }
}

// Scan I2C bus to find active devices
void scanI2CBus() {
    printf("Scanning I2C bus...\\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (testI2CConnection(addr)) {
            printf("Found device at address: 0x%02X\\n", addr);
        }
    }
    printf("I2C scan complete\\n");
}



// TIM handles

// Lifter motors and sensors
DC_motor leftWheel = {&htim3, GPIOD, GPIO_PIN_14, &htim15, TIM_CHANNEL_1, 4, 20, 1};
DC_motor rightWheel  = {&htim23, GPIOD, GPIO_PIN_11, &htim15, TIM_CHANNEL_2, 4, 20, 1};
DC_motor front_lifter = {&htim8, GPIOD, GPIO_PIN_9, &htim16, TIM_CHANNEL_1, 4, 20, 1};
DC_motor back_lifter = {&htim8, GPIOD, GPIO_PIN_10, &htim17, TIM_CHANNEL_1, 4, 20, 1};

VL53L0X_Enhanced backSensor(GPIOG, GPIO_PIN_1, 0x30, &hi2c1, "Back_Lifter");   // Back sensor: XSHUT pin GPIO_PIN_1, I2C address 0x30
VL53L0X_Enhanced frontSensor(GPIOG, GPIO_PIN_2, 0x31, &hi2c1, "Front_Lifter"); // Front sensor: XSHUT pin GPIO_PIN_2, I2C address 0x31

// State variables
bool Direction[2] = {0, 0};
bool lifterMoveState[2] = {1, 1};
int goalHeight[2] = {0, 0};
bool achieve[2] = {0, 0};
int Lifter[2] = {0,1};
float wheel_sp = 0.0f;

// Function implementations
void lifterSetup() {
    setup_step = 1; // Motors
    
    // Initialize motors first
    front_lifter.setup();
    back_lifter.setup();
    rightWheel.setup();
    leftWheel.setup();
    leftWheel.set_motor_parameter(64,100);
    rightWheel.set_motor_parameter(64,100);
//
//    setup_step = 2; // I2C delay
//    HAL_Delay(500); // Longer wait for I2C to be ready
//
//    setup_step = 3; // GPIO reset - avoid address conflicts
//    // CRITICAL: Reset both sensors to avoid I2C address conflicts
//    // Put both sensors in reset state first
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); // Back sensor OFF
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // Front sensor OFF
//    HAL_Delay(100);
//    gpio_test_result = 1; // GPIO test done
//
//    setup_step = 4; // Test I2C bus
//    i2c_bus_test = testI2CBus();
//
//    setup_step = 5; // Init back sensor FIRST
//    // Initialize Back Sensor FIRST (keep front sensor in reset)
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);   // Back sensor ON
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); // Front sensor OFF
//    HAL_Delay(200); // Give time for sensor to boot
//
//    back_sensor_status = 1; // initializing
//
//    // Test I2C connection for back sensor at default address
//    i2c_test_back = testI2CConnection(0x29) ? 1 : -1;
//
//    // Initialize back sensor and change its address to 0x30
//    bool back_init_success = false;
//    for (int attempt = 0; attempt < 3; attempt++) {
//        if (backSensor.Init(VL53_Accuracy::FAST)) {
//            back_init_success = true;
//            break;
//        }
//        HAL_Delay(200);
//        vl53_init_error = attempt + 1; // Track attempts
//    }
//
//    if (back_init_success) {
//        back_sensor_status = 2; // success
//    } else {
//        back_sensor_status = -1; // failed
//    }
//
//    setup_step = 6; // Init front sensor SECOND
//    // Now initialize Front Sensor (back sensor has changed to 0x30)
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET); // Front sensor ON
//    HAL_Delay(200); // Give time for sensor to boot
//
//    front_sensor_status = 1; // initializing
//
//    // Test I2C connection for front sensor at default address
//    i2c_test_front = testI2CConnection(0x29) ? 1 : -1;
//
//    // Initialize front sensor and change its address to 0x31
//    bool front_init_success = false;
//    for (int attempt = 0; attempt < 3; attempt++) {
//        if (frontSensor.Init(VL53_Accuracy::FAST)) {
//            front_init_success = true;
//            break;
//        }
//        HAL_Delay(200);
//    }
//
//    if (front_init_success) {
//        front_sensor_status = 2; // success
//    } else {
//        front_sensor_status = -1; // failed
//    }
//
//    setup_step = 7; // Verification
//    // Final verification of both sensors
//    HAL_Delay(500); // Allow sensors to stabilize
//
//    // Verify back sensor
//    if (back_sensor_status == 2) {
//        i2c_test_back = testI2CConnection(0x30) ? 1 : -1; // Verify at new address
//    }
//
//    // Verify front sensor
//    if (front_sensor_status == 2) {
//        i2c_test_front = testI2CConnection(0x31) ? 1 : -1; // Verify at new address
//    }
//
//    setup_step = 8; // Complete
}

void measureDistance() {
    // Update both sensors and store results for debugging
    if (backSensor.IsInitialized()) {
        backSensor.Update();
        back_lifter_distance = backSensor.GetLastDistance();
        back_sensor_valid = backSensor.IsLastMeasurementValid();
    }
    if (frontSensor.IsInitialized()) {
        frontSensor.Update();
        front_lifter_distance = frontSensor.GetLastDistance();
        front_sensor_valid = frontSensor.IsLastMeasurementValid();
    }
}

// Simple function to test sensors and get readings
void testSensors() {
    // Test both sensors
    if (backSensor.IsInitialized()) {
        backSensor.Update();
    }
    if (frontSensor.IsInitialized()) {
        frontSensor.Update();
    }
}

void lifterMove(int lifter, bool direction, bool lifterMove) {
    if (lifterMove == 1) {
        if (lifter == 1) { // front lifter
            if (direction == 1) { // up
                front_lifter.setDirection(1);
                front_lifter.setPulse(100.f);
            } else if (direction == 0) { // down
                front_lifter.setDirection(0);
                front_lifter.setPulse(100.f);
            }
        } else if (lifter == 0) { // back lifter
            if (direction == 1) { // up
                back_lifter.setDirection(1);
                back_lifter.setPulse(100.f);
            } else if (direction == 0) { // down
                back_lifter.setDirection(0);
                back_lifter.setPulse(100.f);
            }
        }
    } else if (lifterMove == 0) {
        if (lifter == 1) { // stop front lifter
            front_lifter.setPulse(0.f);
        } else if (lifter == 0) { // stop back lifter
            back_lifter.setPulse(0.f);
        }
    }
}


int k = 0;
int lifterWheelSpeed = 2;
void lifterRun() {

    	Direction[0] = k;
    	Direction[1] = k;
        lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
        lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//    } else {
//        // Back lifter control with sensor feedback
//        if (backSensor.IsInitialized() && backSensor.IsLastMeasurementValid()) {
//            uint16_t backDistance = backSensor.GetLastDistance();
//
//            if (Direction[0] == 1) { // back lifter up
//                if (goalHeight[0] > backDistance) {
//                    back_lifter.setDirection(1);
//                    lifterMoveState[0] = 1;
//                    lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
//                } else if (goalHeight[0] <= backDistance) {
//                    lifterMoveState[0] = 0;
//                    lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
//                }
//            } else if (Direction[0] == 0) { // back lifter down
//                if (goalHeight[0] < backDistance) {
//                    back_lifter.setDirection(0);
//                    lifterMoveState[0] = 1;
//                    lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
//                } else if (goalHeight[0] >= backDistance) {
//                    lifterMoveState[0] = 0;
//                    lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
//                }
//            }
//        }
//
//        // Front lifter control with sensor feedback
//        if (frontSensor.IsInitialized() && frontSensor.IsLastMeasurementValid()) {
//            uint16_t frontDistance = frontSensor.GetLastDistance();
//
//            if (Direction[1] == 1) { // front lifter up
//                if (goalHeight[1] > frontDistance) {
//                    front_lifter.setDirection(1);
//                    lifterMoveState[1] = 1;
//                    lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//                } else if (goalHeight[1] <= frontDistance) {
//                    lifterMoveState[1] = 0;
//                    lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//                }
//            } else if (Direction[1] == 0) { // front lifter down
//                if (goalHeight[1] < frontDistance) {
//                    front_lifter.setDirection(0);
//                    lifterMoveState[1] = 1;
//                    lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//                } else if (goalHeight[1] >= frontDistance) {
//                    lifterMoveState[1] = 0;
//                    lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//                }
//            }
//        } else {
//            // Front sensor not available - use manual control
//            lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
//        }
    rightWheel.updateSpeed(-1);
    leftWheel.updateSpeed(-1);
    rightWheel.PI_run();
    leftWheel.PI_run();
    rightWheel.setspeed(lifterWheelSpeed);
    leftWheel.setspeed(lifterWheelSpeed);
    
    }

void setHeight(int lifter, int height) {
    goalHeight[lifter] = height;
}

// Additional helper functions for the enhanced VL53L0X sensors

/**
 * @brief Get current lifter position
 * @param lifter Lifter index (0 = back, 1 = front)
 * @return Current distance measurement in mm, or 0 if invalid
 */
uint16_t getLifterPosition(int lifter) {
    if (lifter == 0) { // back lifter
        if (backSensor.IsInitialized() && backSensor.IsLastMeasurementValid()) {
            return backSensor.GetLastDistance();
        }
    } else if (lifter == 1) { // front lifter
        if (frontSensor.IsInitialized() && frontSensor.IsLastMeasurementValid()) {
            return frontSensor.GetLastDistance();
        }
    }
    return 0; // Invalid or uninitialized
}

/**
 * @brief Check if lifter sensors are working properly
 * @return true if both sensors are initialized and working
 */
bool areLifterSensorsReady() {
    return (backSensor.IsInitialized() && frontSensor.IsInitialized()); // Check both sensors
}

/**
 * @brief Calibrate lifter sensors for precise positioning
 * @param target_distance_mm Target distance for calibration (default 100mm)
 * @return true if both sensors calibrated successfully
 */
bool calibrateLifterSensors(uint16_t target_distance_mm = 100) {
    printf("Starting lifter sensor calibration...\\n");
    printf("Please position targets at %d mm from both sensors\\n", target_distance_mm);
    
    HAL_Delay(3000); // Give time to position targets
    
    bool back_cal_ok = false;
    bool front_cal_ok = false;
    
    if (backSensor.IsInitialized()) {
        back_cal_ok = backSensor.PerformFullCalibration(target_distance_mm);
        printf("Back sensor calibration: %s\\n", back_cal_ok ? "SUCCESS" : "FAILED");
    }
    
    if (frontSensor.IsInitialized()) {
        front_cal_ok = frontSensor.PerformFullCalibration(target_distance_mm);
        printf("Front sensor calibration: %s\\n", front_cal_ok ? "SUCCESS" : "FAILED");
    }
    
    printf("Lifter sensor calibration completed\\n");
    return (back_cal_ok && front_cal_ok); // Both sensors must pass
}

/**
 * @brief Get lifter status information for debugging
 */
void printLifterStatus() {
    printf("=== Lifter Status ===\\n");
    printf("Back Lifter Position: %d mm (Goal: %d mm)\\n", 
           getLifterPosition(0), goalHeight[0]);
    printf("Front Lifter Position: %d mm (Goal: %d mm)\\n", 
           getLifterPosition(1), goalHeight[1]);
    printf("Back Sensor: %s\\n", backSensor.IsInitialized() ? "Ready" : "Not Ready");
    printf("Front Sensor: %s\\n", frontSensor.IsInitialized() ? "Ready" : "Not Ready");
    printf("Back Sensor Valid: %s\\n", backSensor.IsLastMeasurementValid() ? "Yes" : "No");
    printf("Front Sensor Valid: %s\\n", frontSensor.IsLastMeasurementValid() ? "Yes" : "No");
    printf("===================\\n");
}

 // namespace Lifter

/**
 * @brief Simple test function to check both sensors after initialization
 */
void testBothSensors() {
    printf("=== Testing Both VL53L0X Sensors ===\\n");
    
    // Test back sensor
    if (back_sensor_status == 2 && backSensor.IsInitialized()) {
        printf("Back Sensor: INITIALIZED\\n");
        if (backSensor.IsLastMeasurementValid()) {
            printf("Back Distance: %d mm\\n", backSensor.GetLastDistance());
        } else {
            printf("Back Sensor: Invalid measurement\\n");
        }
    } else {
        printf("Back Sensor: FAILED (status: %d)\\n", back_sensor_status);
    }
    
    // Test front sensor
    if (front_sensor_status == 2 && frontSensor.IsInitialized()) {
        printf("Front Sensor: INITIALIZED\\n");
        if (frontSensor.IsLastMeasurementValid()) {
            printf("Front Distance: %d mm\\n", frontSensor.GetLastDistance());
        } else {
            printf("Front Sensor: Invalid measurement\\n");
        }
    } else {
        printf("Front Sensor: FAILED (status: %d)\\n", front_sensor_status);
    }
    
    printf("I2C Tests - Back: %s, Front: %s\\n", 
           (i2c_test_back == 1) ? "OK" : "FAIL",
           (i2c_test_front == 1) ? "OK" : "FAIL");
    printf("Setup completed at step: %d\\n", setup_step);
    printf("================================\\n");
}










