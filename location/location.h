/*
 * location.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_LOCATION_H_
#define INC_LOCATION_H_

#include "DC_motor.h"
#include "pathsensor.h"
#include "chassis.h"
#include "math.h"
#include <cmath>
#include <stdlib.h>

void set_move_mode(int mode);
bool achieve();
bool ach();
bool mission_complete();
bool arrive_destination();
void stop();

void move_mode(int mode) ;

//moving function
void integral_move();//1
void followPath(int path_dir) ;
void path_moveDis(); //2
void direct_move(int direction);
void directMove_findLine();
void direct_moveDistance();
void path_findLine();
void direct_move_find_line();
void lifter_move();//6
void moveDistance(); // 新增：等速率移動功能

//update move mode
void integral_move_to (float x,float y,float w);
void setPath_distance(float path_distance,int _path_dir, float speed);
void set_directMove_findLine(int _line_type, int _dir);
void set_directMove(int direction, float _distance, float _angle);
void setPath_finding_line(int _line_type, int _path_dir, float speed);
void setLifter(int frontLifter,int backLifter,float chassisSP);
void setmoveDistance(float x_dis, float y_dis ,float angle ,float speed,int _accel); // angle: 絕對目標角度（度）
void waitMissionComplete(int _stage);


void chassis_move();

// ============ 全域變數宣告 ============
// 目標位置相關
extern float goal_x, goal_y, goal_theta;
extern float last_x, last_y, last_theta;

// 移動模式和狀態
extern int moveMode_flag; // 0: move to a point, 1: move straight, 2: follow path for a distance, 3: follow path
extern bool achieve_flag;
extern bool mission_flag;

// 控制參數
extern float pos_threshold; // Threshold for position accuracy
extern const float kp_pos;  // Proportional gain for position control
extern const float kp_theta; // Proportional gain for orientation control

// 速度相關變數
extern float move_speed; // 路徑跟隨的速度 (pathsensor.cpp 中定義)
extern float original_move_speed; // 儲存原始速度
extern float max_speed; // Maximum speed for the chassis

// 移動距離和方向
extern float travel_distance; // Distance traveled since the last update
extern int dir; // Direction for direct move, default is 0 (forward)
extern float path_dis; // Distance to follow the path

// 線條類型和路徑方向
extern int line_type;
extern int path_dir; // Direction for path following, default is 0 (forward)

// direct_move 相關變數
extern float distance; // Distance to move in direct_move
extern float angle; // Angle to rotate in direct_move

// moveDistance 相關變數
extern float x_distance; // X 方向需要移動的距離
extern float y_distance; // Y 方向需要移動的距離
extern float dis; // 總移動距離
extern float moveDistance_speed; // 等速率移動的速度 (m/s)
extern float moveangle; // 目標角度 (radians)
extern bool accel; // 是否啟用加速和減速
extern float acceleration_distance; // 加速階段的距離
extern float initial_speed; // 進入函數時的初始速度
extern float current_move_speed; // 當前移動速度

#endif /* INC_LOCATION_H_ */
