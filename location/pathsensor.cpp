/*
 * pathsensor.cpp
 *
 *  Created on: Jul 4, 2025
 *      Author: 88698
 */
#include "pathsensor.h"
#include "location.h"
#include "script.h"
#include "stm32h7xx_hal.h"

extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern float map_x, map_y, last_x, last_y;
extern float cmd_v_x, cmd_v_y, cmd_v_w;
extern bool arrive;

int normal_Speed = 16;
#define w_kp 0.4
#define w_kd 0
#define boundry 2000
#define spin_sp 10

uint16_t adcRead_ADC3[12];
uint16_t adcRead_ADC1[3];
uint32_t adc_value;
int   check = 0;
float weight_err;
float weight_lasttime = 0;
float weight_change = 0;
float tempSpeed[2];
float path_motor_speed[2];
//extern float path_dis ;
int o = 0;
/*
adcRead_ADC3[0]  adc3-0   PC2  right
adcRead_ADC3[1]  adc3-1   PC3    |
adcRead_ADC3[2]  adc3-2   PF3    |
adcRead_ADC3[3]  adc3-3   PA1    V
adcRead_ADC3[4]  adc3-4   PA0  left
adcRead_ADC3[5]  adc3-5   PB0  middle right
adcRead_ADC3[6]  adc3-6   PB1  middle left
*/
void path_setup(){
	//if(HAL_ADC_Start_DMA(&hadc3,(uint32_t *)adcRead_ADC3,7) != HAL_OK)
	// Start DMA for ADC3, storing 12 channels in adcRead_ADC3_ADC3

	//    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcRead_ADC3, 12);
	    // Start DMA for ADC1, storing 3 channels in adcRead_ADC3_ADC1
	  //  HAL_ADCEx_Calibration_Start(&hadc1);
	    //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRead_ADC1, 3);//	    HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&hadc1.Instance->DR, (uint32_t)adcRead_ADC1, 3);
	   // HAL_ADC_Start(&hadc1);
	    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcRead_ADC3, 12);
		check++;
}
void weight(int dir) {//0:front,1:back,2:right,3:,left
    // Calculate line following error (P and D)
	if (dir == 0){//front
    weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[2] + 3*adcRead_ADC3[3]) /
                 (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[2] + adcRead_ADC3[3]));
    weight_change = weight_err - weight_lasttime;
    weight_lasttime = weight_err;

    // For mecanum: output chassis velocity vector
    cmd_v_y = normal_Speed; // Forward speed (positive: forward)
    cmd_v_x = 0;            // No strafe (add logic here if you want to strafe)
    cmd_v_w = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
	}
	else if(dir == 1){//back
	weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[3] + 3*adcRead_ADC3[4]) /
		            (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[3] + adcRead_ADC3[4]));
	weight_change = weight_err - weight_lasttime;
	weight_lasttime = weight_err;

    // For mecanum: output chassis velocity vector
	cmd_v_y = -normal_Speed; // Forward speed (positive: forward)
	cmd_v_x = 0;            // No strafe (add logic here if you want to strafe)
	cmd_v_w = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
	}
	else if(dir == 2){//right
    weight_err = ((float)(-3*adcRead_ADC3[4] - adcRead_ADC3[5] + adcRead_ADC3[6] + 3*adcRead_ADC3[7]) /
						   (adcRead_ADC3[4] + adcRead_ADC3[5]+ 4096 +adcRead_ADC3[6] + adcRead_ADC3[7]));
	weight_change = weight_err - weight_lasttime;
	weight_lasttime = weight_err;

				// For mecanum: output chassis velocity vector
	cmd_v_y = 0; // Forward speed (positive: forward)
	cmd_v_x = normal_Speed;            // No strafe (add logic here if you want to strafe)
	cmd_v_w = (weight_err * w_kp + weight_change * w_kd); // Rotation correction

	}
	else if(dir == 3){//left
	weight_err = ((float)(-3*adcRead_ADC3[8] - adcRead_ADC3[9] + adcRead_ADC3[10] + 3*adcRead_ADC3[11]) /
				     (adcRead_ADC3[8] + adcRead_ADC3[9]+ 4096 +adcRead_ADC3[10] + adcRead_ADC3[11]));
	weight_change = weight_err - weight_lasttime;
	weight_lasttime = weight_err;

		  // For mecanum: output chassis velocity vector
	cmd_v_y = 0; // Forward speed (positive: forward)
	cmd_v_x = -normal_Speed;            // No strafe (add logic here if you want to strafe)
	cmd_v_w = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
	}

	}


//motor_speed[0]:right motor speed, motor_speed[1]:left motor speed
void path(int dir) { // follow path for mecanum chassis

    //turn right (in place)
//   if (adcRead_ADC3[5] >= boundry && adcRead_ADC3[6] < boundry && adcRead_ADC3[0] < boundry && adcRead_ADC3[1] < boundry
//       && adcRead_ADC3[2] < boundry && adcRead_ADC3[3] < boundry && adcRead_ADC3[4] < boundry) {
//
//       cmd_v_y = 0;
//       cmd_v_x = 0;
//       cmd_v_w = -spin_sp;
//
//       //while (adcRead_ADC3[2] < 3 * boundry) {}
//
//       ach(1);
//
//       cmd_v_y = 0;
//       cmd_v_x = 0;
//       cmd_v_w = 0;
//   }
//   // turn left (in place)
//   else if (adcRead_ADC3[5] < boundry && adcRead_ADC3[6] >= 2 * boundry && adcRead_ADC3[0] < boundry && adcRead_ADC3[1] < boundry
//       && adcRead_ADC3[2] < boundry && adcRead_ADC3[3] < boundry && adcRead_ADC3[4] < boundry) {
//       cmd_v_y = 0;
//       cmd_v_x = 0;
//       cmd_v_w = spin_sp;
//
//      // while (adcRead_ADC3[2] < 3 * boundry) {}
//
//       ach(1);
//
//       cmd_v_y = 0;
//       cmd_v_x = 0;
//       cmd_v_w = 0;
//   }
   // forward (line following)
//      if(adcRead_ADC3[5] <= boundry && adcRead_ADC3[6] <= boundry && adcRead_ADC3[0] < boundry && adcRead_ADC3[1] < boundry
//	   && adcRead_ADC3[2] < boundry && adcRead_ADC3[3] < boundry && adcRead_ADC3[4] < boundry) {
//		cmd_v_y = normal_Speed; // Forward speed
//		cmd_v_x = 0;            // No strafe
//		cmd_v_w = 0;            // No rotation
//   }
//    else {
         weight(dir); // sets cmd_v_y, cmd_v_x, cmd_v_w
//    }
}

//go to (x,y)
//void path_move(float path_d){
//
//
//}
//1:front find line, 2:middle find line, 3:find cross road, 4:find line
bool line_check(int type){
	int b = 2000;

	switch(type){
    	case 1://left line
            if(adcRead_ADC3[5] >= b && adcRead_ADC3[6] >= b)
                return 1;
            else
                return 0;
            break; // Note: 'break' after 'return' is redundant but harmless.
	
	    case 2:
            if(adcRead_ADC3[2] >= b && adcRead_ADC3[6] >= b)
                return 1;
            else
                return 0;
            break;
		
	    case 3://left and right
            if(adcRead_ADC3[5] >= b && adcRead_ADC3[6] >= b && adcRead_ADC3[9] >= b && adcRead_ADC3[10] >= b)
                return 1;
            else
                return 0;
            break;
	    case 4://front and left
            if(adcRead_ADC3[1] >= b && adcRead_ADC3[2] >= b && adcRead_ADC3[5] >= b && adcRead_ADC3[6] >= b )
                return 1;
            else
                return 0;
            break;
	    case 5://front and right
            if(adcRead_ADC3[1] >= b && adcRead_ADC3[2] >= b && adcRead_ADC3[9] >= b && adcRead_ADC3[10] >= b)
                return 1;
            else
                return 0;
            break;
	    case 6:
            if(adcRead_ADC3[5] >= b && adcRead_ADC3[6] >= b)
                return 1;
            else
                return 0;
            break;

        // Add a default return value to handle all other cases
        default:
            return 0;
            break;
	}

    // You can also place a single return at the end instead of a default case
    // return 0;
}



