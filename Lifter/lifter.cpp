/*
 * lifter.cpp
 *
 *  Created on: Aug 4, 2025
 *      Author: 88698
 */

#include "lifter.h"
#include "DC_motor.h"
#include "stm32h7xx_hal.h"
#include "VL53.h"
#include "vl53l0x_api.h"



// TIM handles

// Lifter motors and sensors
DC_motor lifter_right_wheel = {&htim8, GPIOD, GPIO_PIN_14, &htim23, TIM_CHANNEL_1, 4, 20, 1};
DC_motor lifter_left_wheel  = {&htim23, GPIOD, GPIO_PIN_11, &htim23, TIM_CHANNEL_2, 4, 20, 1};
DC_motor front_lifter       = {&htim8, GPIOD, GPIO_PIN_9, &htim16, TIM_CHANNEL_1, 4, 20, 1};
DC_motor back_lifter        = {&htim8, GPIOD, GPIO_PIN_10, &htim17, TIM_CHANNEL_1, 4, 20, 1};

VL53L0X backSensor(GPIOG, GPIO_PIN_2, 0x30); // Back sensor: XSHUT pin GPIO_PIN_2, I2C address 0x30
VL53L0X frontSensor(GPIOG, GPIO_PIN_1, 0x31); // Front sensor: XSHUT pin GPIO_PIN_1, I2C address 0x31

// State variables
bool Direction[2] = {0, 0};
bool lifterMoveState[2] = {1, 1};
int goalHeight[2] = {0, 0};
bool achieve[2] = {0, 0};
int Lifter[2] = {0,1};
float wheel_sp = 0.0f;

// Function implementations
void lifterSetup() {
    lifter_right_wheel.setup();
    lifter_left_wheel.setup();
    front_lifter.setup();
    back_lifter.setup();
    backSensor.Init();
    //HAL_Delay(2000);
    frontSensor.Init();

}

void measureDistance() {
	backSensor.Update();
    frontSensor.Update();
    //backSensor.Update();
    //wheel_sp++;
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
void lifterRun() {
	if (k == 0 ){
	lifterMove(Lifter[0],Direction[0],lifterMoveState[0]);
	lifterMove(Lifter[1],Direction[1],lifterMoveState[1]);
	}else{
    if (Direction[0] == 1) { // back lifter up
        if (goalHeight[0] > backSensor.getRangeResult()) { // Use getter function
            back_lifter.setDirection(1);
            lifterMoveState[0] = 1;
            lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
        } else if (goalHeight[0] <= backSensor.getRangeResult()) { // Use getter function
            lifterMoveState[0] = 0;
            lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
        }
    } else if (Direction[0] == 0) { // back lifter down
        if (goalHeight[0] < backSensor.getRangeResult()) { // Use getter function
            back_lifter.setDirection(0);
            lifterMoveState[0] = 1;
            lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
        } else if (goalHeight[0] >= backSensor.getRangeResult()) { // Use getter function
            lifterMoveState[0] = 0;
            lifterMove(Lifter[0], Direction[0], lifterMoveState[0]);
        }
    }

    if (Direction[1] == 1) { // front lifter up
        if (goalHeight[1] > frontSensor.getRangeResult()) { // Use getter function
            front_lifter.setDirection(1);
            lifterMoveState[1] = 1;
            lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
        } else if (goalHeight[1] <= frontSensor.getRangeResult()) { // Use getter function
            lifterMoveState[1] = 0;
            lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
        }
    } else if (Direction[1] == 0) { // front lifter down
        if (goalHeight[1] < frontSensor.getRangeResult()) { // Use getter function
            front_lifter.setDirection(0);
            lifterMoveState[1] = 1;
            lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
        } else if (goalHeight[1] >= frontSensor.getRangeResult()) { // Use getter function
            lifterMoveState[1] = 0;
            lifterMove(Lifter[1], Direction[1], lifterMoveState[1]);
        }
    }
}
}
void setHeight(int lifter, int height) {
    goalHeight[lifter] = height;
}

 // namespace Lifter










