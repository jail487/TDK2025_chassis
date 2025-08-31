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
extern TIM_HandleTypeDef htim14;

extern DC_motor front_lifter,back_lifter;

extern float cmd_v_x,cmd_v_y ,cmd_v_w ;
int ms = 0;
int ttest = 1;

static int ROS_FREQUENCY_Pose = 0;
static int ROS_FREQUENCY_Arrive = 0;
static int ROS_FREQUENCY_SpeedCmd = 0;


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
	path_setup();
	ROS1::init();
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim14);
	Chassis::setup();

	//path_setup();
//	ros_setup();
	//lifterSetup();
}

void loop(void)
{
	//ros_loop();
}


void main_function(){
	//path_setup();

	setup();

	while(1){
		stage_1();
		stage_2();
		break;
		}
//		ROS1::spinCycle();
//		ROS1::hassis_pose();

		//ms++;
		//measureDpub_arrive_destination();
//		ROS1::pub_cistance();


	}



extern int ach_stage ;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM6){

//		if(ttest){
		chassis_move();
//		}else{
//
//		path(path_dir);}
		Chassis::updateSpeed(cmd_v_x,cmd_v_y,cmd_v_w);
		//Lifter::run();
//		move_mo。
		ms++;
		}
	else if (htim -> Instance == TIM7){
		if (++ROS_FREQUENCY_Pose >= ROS_PUB_FREQUENCY) {
					ROS_FREQUENCY_Pose = 0;
					ROS1::pub_chassis_pose();
					ROS1::pub_receive_speed_cmd();
		
	}
}
	else if (htim -> Instance == TIM14){
		if (++ROS_FREQUENCY_Arrive >= ROS_PUB_FREQUENCY) {
					ROS_FREQUENCY_Arrive = 0;
					ROS1::pub_arrive_destination();
		}
	}
	else if (htim -> Instance == TIM13){
		ROS1::spinCycle();

	   // path();
//		lifter_measuredistance();
	    //test++;
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

int ee = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		ee++;
		//處理接收到的資料
	}
}



