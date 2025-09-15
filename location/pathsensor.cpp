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
// 注意：map_x, map_y 現在在 Chassis namespace 內，透過 Chassis::map_x 等存取
extern float last_x, last_y;
extern float cmd_v_x, cmd_v_y, cmd_v_w;
extern bool arrive;

int normal_Speed = 16;
#define w_kp_y 0.4
#define w_kp_x 0.15
#define w_kd 0
#define boundry 2000
#define spin_sp 10

// 新增：速度平滑控制變數
#define LINE_LOST_THRESHOLD 1000  // 所有傳感器數值低於此閾值時認為失去線條
#define SPEED_CHANGE_RATE 2.0f    // 速度變化率 (每次調用增減的速度)
#define MIN_SPEED_RATIO 0.3f      // 最小速度比例 (相對於 normal_Speed)

static float current_speed_x = 0.0f;   // 當前 X 軸速度
static float current_speed_y = 0.0f;   // 當前 Y 軸速度
static float target_speed_x = 0.0f;    // 目標 X 軸速度
static float target_speed_y = 0.0f;    // 目標 Y 軸速度
static int last_direction = -1;        // 上次的方向
static bool line_lost = false;         // 是否失去線條
static uint32_t direction_change_time = 0; // 方向切換時間

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
// 平滑速度更新函數
void smooth_speed_update(float target_x, float target_y) {
    // 計算速度差值
    float diff_x = target_x - current_speed_x;
    float diff_y = target_y - current_speed_y;
    
    // 限制速度變化率
    if (fabs(diff_x) > SPEED_CHANGE_RATE) {
        current_speed_x += (diff_x > 0) ? SPEED_CHANGE_RATE : -SPEED_CHANGE_RATE;
    } else {
        current_speed_x = target_x;
    }
    
    if (fabs(diff_y) > SPEED_CHANGE_RATE) {
        current_speed_y += (diff_y > 0) ? SPEED_CHANGE_RATE : -SPEED_CHANGE_RATE;
    } else {
        current_speed_y = target_y;
    }
}

// 檢測是否失去線條
bool check_line_lost(int dir) {
    bool lost = false;
    
    if (dir == 0) { // front
        lost = (adcRead_ADC3[0] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[1] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[2] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[3] < LINE_LOST_THRESHOLD);
    }
    else if (dir == 1) { // back
        lost = (adcRead_ADC3[0] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[1] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[3] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[4] < LINE_LOST_THRESHOLD);
    }
    else if (dir == 2) { // right
        lost = (adcRead_ADC3[8] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[9] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[10] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[11] < LINE_LOST_THRESHOLD);
    }
    else if (dir == 3) { // left
        lost = (adcRead_ADC3[4] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[5] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[6] < LINE_LOST_THRESHOLD && 
                adcRead_ADC3[7] < LINE_LOST_THRESHOLD);
    }
    
    return lost;
}

void weight(int dir) {//0:front,1:back,2:right,3:left
    // 檢測方向切換
    bool direction_changed = (last_direction != -1 && last_direction != dir);
    if (direction_changed) {
        direction_change_time = HAL_GetTick();
        // 保存當前速度作為初始速度
        current_speed_x = cmd_v_x;
        current_speed_y = cmd_v_y;
    }
    last_direction = dir;
    
    // 檢測是否失去線條
    line_lost = check_line_lost(dir);
    
    // 如果失去線條，降低速度繼續前進
    if (line_lost) {
        float min_speed = normal_Speed * MIN_SPEED_RATIO;
        
        if (dir == 0) {        // front
            target_speed_y = min_speed;
            target_speed_x = 0;
        }
        else if (dir == 1) {   // back
            target_speed_y = -min_speed;
            target_speed_x = 0;
        }
        else if (dir == 2) {   // right
            target_speed_y = 0;
            target_speed_x = min_speed;
        }
        else if (dir == 3) {   // left
            target_speed_y = 0;
            target_speed_x = -min_speed;
        }
        
        smooth_speed_update(target_speed_x, target_speed_y);
        cmd_v_x = current_speed_x;
        cmd_v_y = current_speed_y;
        cmd_v_w = 0; // 失去線條時不進行角度修正
        return;
    }
    
    // Calculate line following error (P and D)
    if (dir == 0){//front
        weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[2] + 3*adcRead_ADC3[3]) /
                     (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[2] + adcRead_ADC3[3]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        // 設定目標速度
        target_speed_y = normal_Speed;
        target_speed_x = 0;
        
        // 平滑速度更新
        smooth_speed_update(target_speed_x, target_speed_y);
        
        // For mecanum: output chassis velocity vector
        cmd_v_y = current_speed_y; // 使用平滑後的速度
        cmd_v_x = current_speed_x;
        cmd_v_w = (weight_err * w_kp_y + weight_change * w_kd); // Rotation correction
    }
    else if(dir == 1){//back
        weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[3] + 3*adcRead_ADC3[4]) /
                    (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[3] + adcRead_ADC3[4]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        // 設定目標速度
        target_speed_y = -normal_Speed;
        target_speed_x = 0;
        
        // 平滑速度更新
        smooth_speed_update(target_speed_x, target_speed_y);
        
        // For mecanum: output chassis velocity vector
        cmd_v_y = current_speed_y; // 使用平滑後的速度
        cmd_v_x = current_speed_x;
        cmd_v_w = (weight_err * w_kp_y + weight_change * w_kd); // Rotation correction
    }
    else if(dir == 2){//right
        weight_err = ((float)(-3*adcRead_ADC3[8] - adcRead_ADC3[9] + adcRead_ADC3[10] + 3*adcRead_ADC3[11]) /
                           (adcRead_ADC3[8] + adcRead_ADC3[9]+ 4096 +adcRead_ADC3[10] + adcRead_ADC3[11]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        // 設定目標速度
        target_speed_y = 0;
        target_speed_x = normal_Speed;
        
        // 平滑速度更新
        smooth_speed_update(target_speed_x, target_speed_y);
        
        // For mecanum: output chassis velocity vector
        cmd_v_y = current_speed_y; // 使用平滑後的速度
        cmd_v_x = current_speed_x;
        cmd_v_w = (weight_err * w_kp_x + weight_change * w_kd); // Rotation correction
    }
    else if(dir == 3){//left
        weight_err = ((float)(-3*adcRead_ADC3[4] - adcRead_ADC3[5] + adcRead_ADC3[6] + 3*adcRead_ADC3[7]) /
                         (adcRead_ADC3[4] + adcRead_ADC3[5]+ 4096 +adcRead_ADC3[6] + adcRead_ADC3[7]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        // 設定目標速度
        target_speed_y = 0;
        target_speed_x = -normal_Speed;
        
        // 平滑速度更新
        smooth_speed_update(target_speed_x, target_speed_y);
        
        // For mecanum: output chassis velocity vector
        cmd_v_y = current_speed_y; // 使用平滑後的速度
        cmd_v_x = current_speed_x;
        cmd_v_w = (weight_err * w_kp_x + weight_change * w_kd); // Rotation correction
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
	int b = 1000;
	int c= 3000;

	switch(type){
    	case 1://left and right line
            if(adcRead_ADC3[5] >= b && adcRead_ADC3[10] >= b && adcRead_ADC3[1] <= b)
                return 1;
            else
                return 0;
            break; // Note: 'break' after 'return' is redundant but harmless.
	
	    case 2:////front and right line
	    	  if((adcRead_ADC3[1] >= b||adcRead_ADC3[2] >= b) && (adcRead_ADC3[9] >= b||adcRead_ADC3[10] >= b ))
                return 1;
            else
                return 0;
            break;
		
	    case 3://left and right
            if((adcRead_ADC3[5] >= b || adcRead_ADC3[6] >= b )&& (adcRead_ADC3[9] >= b || adcRead_ADC3[10] >= b))
                return 1;
            else
                return 0;
            break;
	    case 4://front and left
            if((adcRead_ADC3[1] >= b||adcRead_ADC3[2] >= b) && (adcRead_ADC3[5] >= b||adcRead_ADC3[6] >= b ))
                return 1;
            else
                return 0;
            break;
	    case 5://left line
            if(adcRead_ADC3[5] >= b || adcRead_ADC3[6] >= b )
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



