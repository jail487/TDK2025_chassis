/*
 * mainpp.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 *
 *
 */
#include "mainpp.h"
#include "ROS1.h"
#include "pathsensor.h"
#include "location.h"
#include "script.h"
#include "DC_motor.h"
#include "chassis.h"
#include "lifter.h"
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim13;

extern float cmd_v_x,cmd_v_y ,cmd_v_w ;
int ms = 0;
int test = 1;

extern int path_dir;

extern int count;



//void Error_Handler(void)
//{
//
//  __disable_irq();
//  while (1)
//  {
//  }
//}



void setup(){
	//ros_setup();
	//path_setup();
//	ROS1::init();
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim13);
	chassis_setup();
	//lifter_setup();
	path_setup();
//	ros_setup();

}

void loop(void)
{
	//ros_loop();
}


void main_function(){
	//path_setup();
	setup();

	while(1){
		//stage_1();
		//stage_2();
		//lifter_measuredistance();

	}
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM6){
//		chassis_move();
		path(path_dir);
		////ROS1::spinCycle();
		//path(path_dir);
		chassis_update_speed(cmd_v_x,cmd_v_y,cmd_v_w);
		//move_mode();
		ms++;
		}
	if (htim -> Instance == TIM13){
	   // path();
//		lifter_measuredistance();
	    test++;
	    }
}

int adc1 = 0,adc3 = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc -> Instance == ADC1){
		adc1++;
	}
	if (hadc -> Instance == ADC3){
		adc3++;
	}
}



