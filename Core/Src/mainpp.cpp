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


int ms = 0;

bool start_flag = false;

static int ROS_FREQUENCY_Pose = 0;
static int ROS_FREQUENCY_Arrive = 0;
static int ROS_FREQUENCY_SpeedCmd = 0;


extern int path_dir;

extern int count;
int ttest = 0;





//void Error_Handler(void)
//{
//
//  __disable_irq();
//  while (1)
//  {
//  }
//}


int pb2 = 0;
void setup(){
	HAL_TIM_Base_Start_IT(&htim7);
	ROS1::init();

	pb2++;


	//ros_setup();
	path_setup();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim14);
	
	// setup() 內部會調用 pinpoint_init() 並設置診斷代碼
	Chassis::setup();
	
	// 等待初始化完成，讓 Live Expressions 顯示結果
	//HAL_Delay(2000);
	
	// 觀察 Live Expressions 中的 device_id:
	// 0xF0 = I2C 未準備
	// 0xF1 = I2C 總線無回應  
	// 0xF2 = I2C 正常但無設備
	// 0xF3 = 找到其他設備但不是 Pinpoint
	// 0x31-0x39 = 找到 Pinpoint 的地址

	//path_setup();
//	ros_setup();
	lifterSetup();
	Chassis::setSpeed(0,0,0);
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
			ROS1::spinCycle();
		}
}

void loop(void)
{
	stage_1();
    stage_2();
	stage_3();
}


void main_function(){
	setup();
	while(1){
		if (ttest == 1){
			testt();
		}
		else{
		loop();
		}
		break;
	}
}




extern int ach_stage ;
int pub = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM6){
		//if (!start_flag) return;

//		if(ttest){
		chassis_move();
		Chassis::updateSpeed(); // 更新底盤系統（包含 Pinpoint 更新）
		ms++;
		}
	else if (htim -> Instance == TIM7){

		ROS1::pub_chassis_pose();
		ROS1::pub_current_stage();
		pub++;


		
		// 重新啟用 Pinpoint，但使用極低頻率 (約 15Hz)
//		static uint32_t imu_update_counter = 0;
//		if (++imu_update_counter >= 80) {  // 100:1 分頻，從 1562Hz 降到 ~15.6Hz
//			imu_update_counter = 0;
//			Chassis::IMU_update();
//		}

	//}mu_update_counter = 0;
			//Chassis::IMU_update();
		//}

	//}
}
//	else if (htim -> Instance == TIM14){
//		if (++ROS_FREQUENCY_Arrive >= ROS_PUB_FREQUENCY) {
//					ROS_FREQUENCY_Arrive = 0;
//					ROS1::pub_arrive_destination();
//		}
//	}
	else if (htim -> Instance == TIM13){


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




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		switch (GPIO_Pin) {
			case GPIO_PIN_2:
				//pb2++;

			if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET){
				start_flag = true;
		  
		  }
				//count++;
				break;
			default:
				// 處理 GPIO_PIN_3 的中斷
				break;
			// 可以根據需要添加更多的 case
		}
	}




