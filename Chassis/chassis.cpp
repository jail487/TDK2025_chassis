/*
 * chassis.cpp
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */
#include <stdint.h>
#include "chassis.h"
#include "DC_motor.h"
#include "stm32h7xx_hal.h" // Ensure this header is included for GPIO definitions
#include <math.h>



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim24;


float chassis_width = 40.69;
float chassis_length = 26.5;
float wheel_diameter = 15.2;//cm
float span = 0.001;
float pi = 3.1415926;
float kp =4.8,ki =10;
float v_x = 0.f,v_y = 0.f,v_w = 0.f;//rps
float map_x = 0.f,map_y = 0.f,theta = 0.f;//global coordinate
extern float cmd_v_y,cmd_v_x,cmd_v_w;//local speed
float world_v_y,world_v_x;//local speed

float cmd_w_max = 1.5;

DC_motor wheel_FL ={&htim1,GPIOG,GPIO_PIN_11,&htim5,TIM_CHANNEL_1,4,20,0};
DC_motor wheel_FR ={&htim2,GPIOG,GPIO_PIN_14,&htim5,TIM_CHANNEL_2,4,20,1};
DC_motor wheel_BL ={&htim8,GPIOG,GPIO_PIN_15,&htim5,TIM_CHANNEL_3,4.0,20,1};
DC_motor wheel_BR ={&htim4,GPIOD,GPIO_PIN_15,&htim5,TIM_CHANNEL_4,4,20,1};


void chassis_setup(){
    wheel_FR.setup();
    wheel_FL.setup();
    wheel_BR.setup();
    wheel_BL.setup();
}


void mecan_IK_transform(float _v_x, float _v_y, float _v_w) {
    // Convert wheel_diameter to radius
    float r = wheel_diameter / 2.0f / 1.0f; // wheel radius in centimeters

    // Chassis geometry factor
    float L = chassis_length / 1.0f;
    float W = chassis_width / 1.0f;
    float a = L + W;

    // Calculate wheel speeds in cm/s
    float v1 = -_v_x + _v_y + _v_w * a / 2.0f; // Front Right
    float v2 =  _v_x + _v_y - _v_w * a / 2.0f; // Front Left
    float v3 =  _v_x + _v_y + _v_w * a / 2.0f; // Back Right
    float v4 = -_v_x + _v_y - _v_w * a / 2.0f; // Back Left

    // Convert linear speed (cm/s) to angular speed (rad/s)
    float w1 = v1 / r / pi;
    float w2 = v2 / r / pi;
    float w3 = v3 / r / pi;
    float w4 = v4 / r / pi;

    wheel_FR.setspeed(w1);
    wheel_FL.setspeed(w2);
    wheel_BR.setspeed(w3);
    wheel_BL.setspeed(w4);
}

void mecan_FK_transform() {
    float v1 = wheel_FR.get_speed()*wheel_diameter*pi;
    float v2 = wheel_FL.get_speed()*wheel_diameter*pi;
    float v3 = wheel_BR.get_speed()*wheel_diameter*pi;
    float v4 = wheel_BL.get_speed()*wheel_diameter*pi;
    v_x = -(v1 - v2 - v3 + v4) / 4.0f;
    v_y = (v1 + v2 + v3 + v4) / 4.0f;
    v_w = -(-v1 + v2 - v3 + v4) / (4.0f * (chassis_width + chassis_length) / 2.0f);
}


void localization() {
    // 方向角轉換用
    float cos_t = cos(theta);
    float sin_t = sin(theta);

    // 機體座標轉換成世界座標再積分
    map_x += (v_x * cos_t - v_y * sin_t) * span;
    map_y += (v_x * sin_t + v_y * cos_t) * span;
    theta += v_w * span;

    // 可選：角度標準化在 -π ~ π，避免θ無限增長
    if (theta > M_PI) {
        theta -= 2 * pi;
    } else if (theta < -M_PI) {
        theta += 2 * pi;
    }
}

void transfer_to_localspeed(){

	float cos_t = cos(theta);
	float sin_t = sin(theta);

	cmd_v_x = (world_v_x * cos(theta)) + (world_v_y * sin(theta));
	cmd_v_y = (-world_v_x * sin(theta)) + (world_v_y * cos(theta));
}

void chassis_update_speed(float _v_x,float _v_y,float _v_w){
    mecan_IK_transform(_v_x,_v_y,_v_w);
    wheel_FR.PI_run();
    wheel_FL.PI_run();
    wheel_BR.PI_run();
    wheel_BL.PI_run();
    wheel_FR.update_speed(1);
    wheel_FL.update_speed(-1);
    wheel_BR.update_speed(1);
    wheel_BL.update_speed(-1);
    mecan_FK_transform();
    localization();
}

//void chassis_task(){
////	if(x<300)cmd_v_x =0.2;
////	else if (y>=300)cmd_v_x = 0;
////	if(y<300)cmd_v_y =1;
////	else if (y>=300)cmd_v_y = 0;
////	if(theta<10*pi)cmd_v_w = pi/4;
////	else if(theta>=20*pi)cmd_v_w = 0;
//
//}



