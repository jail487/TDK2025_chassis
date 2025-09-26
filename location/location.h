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

// ============ 統一速度控制系統 ============
struct SpeedProfile {
    float max_speed;           // 最大線速度 (m/s)
    float acceleration;        // 線性加速度 (m/s²)
    float deceleration;        // 線性減速度 (m/s²)
    float jerk_limit;          // 線性加加速度限制 (m/s³)
    float min_speed_ratio;     // 最小速度比例 (0.0-1.0)
    
    // 新增：專門的角速度控制參數
    float max_angular_speed;      // 最大角速度 (rad/s)
    float angular_acceleration;   // 角加速度 (rad/s²)  
    float angular_deceleration;   // 角減速度 (rad/s²)
    float angular_jerk_limit;     // 角加加速度限制 (rad/s³)
    float angular_speed_ratio;    // 角速度相對於線速度的比例 (預設 0.1)
};

class SpeedController {
private:
    SpeedProfile profile;
    float current_speed_x, current_speed_y, current_speed_w;
    float target_speed_x, target_speed_y, target_speed_w;
    uint32_t last_update_time;
    bool smooth_enabled;
    
public:
    // 建構函式
    SpeedController();
    
    // 設定速度配置
    void setProfile(const SpeedProfile& new_profile);
    void setMaxSpeed(float max_speed);
    void setAcceleration(float accel, float decel = -1.0f);
    void setSmoothEnabled(bool enabled);
    
    // 新增：專門的角速度控制函式
    void setAngularSpeed(float max_angular_speed);
    void setAngularAcceleration(float angular_accel, float angular_decel = -1.0f);
    void setAngularSpeedRatio(float ratio); // 設定角速度相對線速度的比例
    
    // 速度計算和控制
    void setTargetSpeed(float vx, float vy, float vw = 0.0f);
    void updateSpeed();
    void getCurrentSpeed(float& vx, float& vy, float& vw);
    
    // 專用控制函式
    void calculateTrajectorySpeed(float distance_remaining, float total_distance);
    void calculateRotationSpeed(float angle_remaining, float total_angle);
    void calculateProportionalSpeed(float dx, float dy, float dtheta);
    
    // 停止和重設
    void stop();
    void reset();
    
    // 查詢狀態
    bool isAtTarget(float threshold = 0.01f);
    float getSpeedNorm();
    
    // 新增：公開的存取函式
    float getMaxSpeed() const { return profile.max_speed; }
    float getMinSpeedRatio() const { return profile.min_speed_ratio; }
    const SpeedProfile& getProfile() const { return profile; }
};

// 全域速度控制器實例
extern SpeedController g_speed_controller;

void set_move_mode(int mode);
bool achieve();
bool ach();
bool mission_complete();
bool arrive_destination();
void stop();

// 統一速度控制系統初始化
void initSpeedController();
void configureSpeedProfile(float max_speed, float acceleration, float deceleration = -1.0f);
// 新增：專門的角速度配置函式
void configureAngularSpeedProfile(float max_angular_speed, float angular_acceleration, float angular_deceleration = -1.0f);

void move_mode(int mode) ;

//moving function
void integral_move();//1
void followPath(int path_dir) ;
void path_moveDis(); //2
void direct_move(int direction, float speed);
void directMove_findLine();
void direct_moveDistance();
void path_findLine();
void direct_move_find_line();
void lifter_move();//6
void moveDistance(); // 新增：等速率移動功能

//update move mode
void integral_move_to (float x,float y,float w);
void setPath_distance(float path_distance,int _path_dir, float speed);
void set_directMove_findLine(int _dir, int _line_type, float speed);
void set_directMove(int direction, float _distance, float _angle, float speed);
void setPath_finding_line(int _line_type, int _path_dir, float speed);
void setLifter(int frontLifter,int backLifter,float chassisSP);
void setmoveDistance(float x_dis, float y_dis ,float angle ,float speed,int _accel, int rotation_dir = 1); // angle: 目標角度（度）, rotation_dir: 旋轉方向控制
void waitMissionComplete(int _stage);

// ============ 尋跡參數調整函式 ============
void setPathfindingAcceleration(float speed_change_rate);  // 設定循跡加減速率
void setPathfindingMinSpeed(float min_speed_ratio);        // 設定循跡最小速度比例
void setPathfindingThreshold(int line_lost_threshold);     // 設定失去線條的閾值
void getPathfindingParams(float* speed_rate, float* min_ratio, int* threshold); // 獲取當前參數

void chassis_move();
void updatePosition(float x, float y, float z);

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
extern float normal_Speed;
extern float direct_move_speed;

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
//extern int currentStage;

#endif /* INC_LOCATION_H_ */
