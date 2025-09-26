/*
 * chassis.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_

#include <stdint.h>  // 包含 uint8_t, uint32_t 等定義

// === Chassis Physical Parameters ===
extern float chassis_width;
extern float chassis_length;
extern float wheel_diameter;
extern float span;

// === Control Parameters ===
extern float kp, ki;
extern float KP_T, KI_T;
extern float cmd_w_max;

// === Position and Orientation (Encoder-based) ===
extern float map_x, map_y, theta;  // encoder-based positioning (cm, cm, rad)

// === Velocity Variables ===
// Real speeds from encoders (used by SpeedController)
extern float v_x, v_y, v_w;  // rps
// Command speeds (local coordinate)
extern float cmd_v_x, cmd_v_y, cmd_v_w;  // local speed
// World coordinate speeds
extern float world_v_x, world_v_y;  // world speed

// === Pinpoint Status Variables ===
extern bool pinpoint_initialized;
extern bool pinpoint_ready;
extern volatile bool pinpoint_dma_enabled;
extern uint32_t pinpoint_last_dma_start;
extern uint32_t pinpoint_dma_interval;

// === Mathematical Constants ===
#define pi 3.14159265358979323846

namespace Chassis {

void setup();
void mecan_IK_transform(float _v_x,float _v_y,float _v_w);
void mecan_FK_transform();
void localization();
void updateSpeed();
void setSpeed(float _v_x, float _v_y, float _v_w);  // 設定速度指令
void chassis_task();



}  // end namespace Chassis









#endif /* CHASSIS_H_ */
