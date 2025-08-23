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

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim24;
extern TIM_HandleTypeDef htim15;

DC_motor lifter_right_wheel = {&htim8, GPIOD, GPIO_PIN_14, &htim15, TIM_CHANNEL_1, 4, 20, 1};
DC_motor lifter_left_wheel  = {&htim23, GPIOD, GPIO_PIN_11, &htim15, TIM_CHANNEL_2, 4, 20, 1};
DC_motor front_lifter       = {&htim8, GPIOD, GPIO_PIN_9, &htim24, TIM_CHANNEL_1, 4, 20, 1};
DC_motor back_lifter        = {&htim8, GPIOD, GPIO_PIN_10, &htim24, TIM_CHANNEL_2, 4, 20, 1};

VL53L0X front_sensor = {GPIOG, GPIO_PIN_1, 0};
VL53L0X back_sensor = {GPIOG, GPIO_PIN_2, 1};

void lifter_setup() {
    lifter_right_wheel.setup();
    lifter_left_wheel.setup();
    front_lifter.setup();
    back_lifter.setup();

    front_sensor.Init();
    back_sensor.Init();
}

void lifter_measuredistance() {
    front_sensor.Update();
    back_sensor.Update();
} // Closing brace for the function}





