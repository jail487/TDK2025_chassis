/*
 * location.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 *
         front
           90
           y
            ︿
           |
           |
           |
 180--------------->x 0
           |
           |
           |
          -90
 */
#include "location.h"
#include "lifter.h"
#include "pathSensor.h"
#include "chassis.h"
#include "stdlib.h"
#include "cmath"
#include <math.h>
#include "ros1.h"
#include "stm32h7xx_hal.h"  // 新增：包含 HAL_GetTick() 的定義
extern int ttest;


#define pi 3.14159265358979323846
extern int ach_stage;
extern float path_dis, path_motor_speed[2];

// ============ 統一速度控制系統實現 ============
SpeedController g_speed_controller;

// SpeedController 建構函式
SpeedController::SpeedController() {
    // 預設線性速度配置
    profile.max_speed = 40.0f;        // 16 cm/s 或適當單位
    profile.acceleration = 10.0f;      // 8 cm/s²
    profile.deceleration = 12.0f;     // 12 cm/s² (減速比加速快)
    profile.jerk_limit = 20.0f;       // 20 cm/s³
    profile.min_speed_ratio = 0.4f;   // 最小10%速度
    
    // 新增：預設角速度配置
    profile.max_angular_speed = 0.8f;     // 0.3 rad/s (約 17度/秒) - 降低避免振盪
    profile.angular_acceleration = 0.8f;  // 0.3 rad/s²  - 降低加速度
    profile.angular_deceleration = 0.8f;  // 0.8 rad/s² - 較強的減速
    profile.angular_jerk_limit = 3.0f;    // 3 rad/s³
    profile.angular_speed_ratio = 0.3f;   // 角速度為線速度的10%
    
    // 初始化狀態
    current_speed_x = current_speed_y = current_speed_w = 0.0f;
    target_speed_x = target_speed_y = target_speed_w = 0.0f;
    last_update_time = HAL_GetTick();
    smooth_enabled = true;
}


void SpeedController::setProfile(const SpeedProfile& new_profile) {
    profile = new_profile;
}

void SpeedController::setMaxSpeed(float max_speed) {
    profile.max_speed = max_speed;
}

void SpeedController::setAcceleration(float accel, float decel) {
    profile.acceleration = accel;
    profile.deceleration = (decel > 0) ? decel : accel * 1.5f; // 預設減速比加速快50%
}

void SpeedController::setSmoothEnabled(bool enabled) {
    smooth_enabled = enabled;
}

// 新增：專門的角速度控制函式
void SpeedController::setAngularSpeed(float max_angular_speed) {
    profile.max_angular_speed = max_angular_speed;
}

void SpeedController::setAngularAcceleration(float angular_accel, float angular_decel) {
    profile.angular_acceleration = angular_accel;
    profile.angular_deceleration = (angular_decel > 0) ? angular_decel : angular_accel * 1.5f;
}

void SpeedController::setAngularSpeedRatio(float ratio) {
    profile.angular_speed_ratio = ratio;
}

void SpeedController::setTargetSpeed(float vx, float vy, float vw) {

    target_speed_x = vx;
    target_speed_y = vy;
    target_speed_w = vw;
    
    // 限制速度在最大值內
    float speed_norm = sqrt(vx * vx + vy * vy);
    if (speed_norm > profile.max_speed) {
        float scale = profile.max_speed / speed_norm;
        target_speed_x *= scale;
        target_speed_y *= scale;
    }    
    // 限制角速度 - 使用專門的角速度參數
    if (fabs(target_speed_w) > profile.max_angular_speed) {
        target_speed_w = (target_speed_w > 0) ? profile.max_angular_speed : -profile.max_angular_speed;
    }
}

void SpeedController::updateSpeed() {
    if (!smooth_enabled) {
        current_speed_x = target_speed_x;
        current_speed_y = target_speed_y;
        current_speed_w = target_speed_w;
        Chassis::setSpeed(current_speed_x, current_speed_y, current_speed_w);
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update_time) / 1000.0f; // 轉換為秒
    last_update_time = current_time;
    
    // 防止dt過大或過小
    if (dt > 0.1f) dt = 0.1f;
    if (dt < 0.001f) dt = 0.001f;
    
    // 計算速度差值
    float dx_speed = target_speed_x - current_speed_x;
    float dy_speed = target_speed_y - current_speed_y;
    float dw_speed = target_speed_w - current_speed_w;
    
    // 計算最大允許的速度變化量
    float max_accel_change = profile.acceleration * dt;
    float max_decel_change = profile.deceleration * dt;
    
    // 計算最大允許的角速度變化量
    float max_angular_accel_change = profile.angular_acceleration * dt;
    float max_angular_decel_change = profile.angular_deceleration * dt;
    
    // X軸速度更新
    if (fabs(dx_speed) > max_accel_change) {
        float max_change = (dx_speed > 0) ? max_accel_change : -max_accel_change;
        // 如果是減速，使用更大的變化量
        if ((dx_speed > 0 && current_speed_x > target_speed_x) || 
            (dx_speed < 0 && current_speed_x < target_speed_x)) {
            max_change = (dx_speed > 0) ? max_decel_change : -max_decel_change;
        }
        current_speed_x += max_change;
    } else {
        current_speed_x = target_speed_x;
    }
    
    // Y軸速度更新
    if (fabs(dy_speed) > max_accel_change) {
        float max_change = (dy_speed > 0) ? max_accel_change : -max_accel_change;
        if ((dy_speed > 0 && current_speed_y > target_speed_y) || 
            (dy_speed < 0 && current_speed_y < target_speed_y)) {
            max_change = (dy_speed > 0) ? max_decel_change : -max_decel_change;
        }
        current_speed_y += max_change;
    } else {
        current_speed_y = target_speed_y;
    }
    
    // 角速度更新 - 使用專門的角速度加減速參數
    if (fabs(dw_speed) > max_angular_accel_change) {
        float max_angular_change = (dw_speed > 0) ? max_angular_accel_change : -max_angular_accel_change;
        // 如果是角減速，使用更大的變化量
        if ((dw_speed > 0 && current_speed_w > target_speed_w) || 
            (dw_speed < 0 && current_speed_w < target_speed_w)) {
            max_angular_change = (dw_speed > 0) ? max_angular_decel_change : -max_angular_decel_change;
        }
        current_speed_w += max_angular_change;
    } else {
        current_speed_w = target_speed_w;
    }
    
    // 輸出到底盤控制
    Chassis::setSpeed(current_speed_x, current_speed_y, current_speed_w);
}

void SpeedController::getCurrentSpeed(float& vx, float& vy, float& vw) {
    vx = current_speed_x;
    vy = current_speed_y;
    vw = current_speed_w;
}

void SpeedController::calculateTrajectorySpeed(float distance_remaining, float total_distance) {
    if (total_distance <= 0) return;
    
    float progress = 1.0f - (distance_remaining / total_distance);
    float speed_scale = 1.0f;
    
    // 三階段速度控制：加速(30%) - 恆速(40%) - 減速(30%)
    if (progress < 0.3f) {
        // 加速階段 - 使用S曲線
        float accel_progress = progress / 0.3f;
        speed_scale = profile.min_speed_ratio + 
                     (1.0f - profile.min_speed_ratio) * (0.5f * (1.0f - cos(accel_progress * pi)));
    } else if (progress > 0.7f) {
        // 減速階段 - 使用S曲線
        float decel_progress = (progress - 0.7f) / 0.3f;
        speed_scale = profile.min_speed_ratio + 
                     (1.0f - profile.min_speed_ratio) * (0.5f * (1.0f + cos(decel_progress * pi)));
    } else {
        // 恆速階段
        speed_scale = 1.0f;
    }
    
    // 確保速度不低於最小值
    speed_scale = fmax(speed_scale, profile.min_speed_ratio);
    
    // 更新目標速度（保持當前方向，只調整大小）
    float current_norm = sqrt(target_speed_x * target_speed_x + target_speed_y * target_speed_y);
    if (current_norm > 0.01f) {
        target_speed_x = (target_speed_x / current_norm) * profile.max_speed * speed_scale;
        target_speed_y = (target_speed_y / current_norm) * profile.max_speed * speed_scale;
    }
}

void SpeedController::calculateRotationSpeed(float angle_remaining, float total_angle) {
    if (total_angle <= 0) return;
    
    float progress = 1.0f - (fabs(angle_remaining) / total_angle);
    float speed_scale = 1.0f;
    
    // 旋轉的三階段控制
    if (progress < 0.3f) {
        float accel_progress = progress / 0.3f;
        speed_scale = 0.2f + 0.8f * (0.5f * (1.0f - cos(accel_progress * pi)));
    } else if (progress > 0.7f) {
        float decel_progress = (progress - 0.7f) / 0.3f;
        speed_scale = 0.2f + 0.8f * (0.5f * (1.0f + cos(decel_progress * pi)));
    }
    
    float max_angular_speed = profile.max_angular_speed;
    target_speed_w = (angle_remaining > 0) ? (max_angular_speed * speed_scale) : (-max_angular_speed * speed_scale);
}

void SpeedController::calculateProportionalSpeed(float dx, float dy, float dtheta) {
    // PID控制的P項
    const float kp_pos = 0.5f;
    const float kp_theta = 1.0f; // 降低角度增益，避免震盪
    
    target_speed_x = kp_pos * dx;
    target_speed_y = kp_pos * dy;
    target_speed_w = kp_theta * dtheta;
    
    // 自動限速
    setTargetSpeed(target_speed_x, target_speed_y, target_speed_w);
}

void SpeedController::stop() {
    target_speed_x = target_speed_y = target_speed_w = 0.0f;
    if (!smooth_enabled) {
        current_speed_x = current_speed_y = current_speed_w = 0.0f;
        Chassis::setSpeed(0, 0, 0);
    }
}

void SpeedController::reset() {
    current_speed_x = current_speed_y = current_speed_w = 0.0f;
    target_speed_x = target_speed_y = target_speed_w = 0.0f;
    last_update_time = HAL_GetTick();
}

bool SpeedController::isAtTarget(float threshold) {
    float speed_diff = sqrt(pow(target_speed_x - current_speed_x, 2) + 
                           pow(target_speed_y - current_speed_y, 2) + 
                           pow(target_speed_w - current_speed_w, 2));
    return speed_diff < threshold;
}

float SpeedController::getSpeedNorm() {
    return sqrt(current_speed_x * current_speed_x + current_speed_y * current_speed_y);
}
//extern int mission_complete ;
//// direct_move 相關變數
float distance = 0.f; // Distance to move in direct_move
float angle = 0.f; // Angle to rotate in direct_move
float direct_move_speed = 5.0f; // Speed for set_directMove

// moveDistance 相關變數
float x_distance = 0.f; // X 方向需要移動的距離
float y_distance = 0.f; // Y 方向需要移動的距離
float dis = 0.f; // 總移動距離
float moveDistance_speed = 5.f; // 等速率移動的速度 (m/s)
float moveangle = 0.f; // 目標角度 (radians)
bool accel = false; // 是否啟用加速和減速
float acceleration_distance = 0.f; // 加速階段的距離
float initial_speed = 0.f; // 進入函數時的初始速度
float current_move_speed = 0.f; // 當前移動速度p_y, theta 現在在全域範圍，可直接存取
extern uint32_t adcRead[7];

// 新增：從 pathsensor.cpp 移植過來的外部變數
extern uint16_t adcRead_ADC3[12];    // ADC 讀取陣列
extern float weight_err;             // 權重誤差
extern float weight_lasttime;        // 上次權重值
extern float weight_change;          // 權重變化量

// 全域變數定義（移除SpeedController，由chassis.cpp統一管理速度）
// 目標位置相關
float goal_x = 0, goal_y = 0, goal_theta = 0;
float last_x = 0, last_y = 0, last_theta = 0;

// 移動模式和狀態
int moveMode_flag = 0; // 0: move to a point, 1: move straight, 2: follow path for a distance, 3: follow path
bool achieve_flag = false;
bool mission_flag = true;

// 控制參數
float pos_threshold = 0.1; // Threshold for position accuracy
const float theta_threshold = 0.05; // 放寬角度閾值到約2.86度，避免無法停止
const float kp_pos = 0.5;  // Proportional gain for position control
const float kp_theta = 10; // Proportional gain for orientation control

// 速度相關變數 (move_speed 在 pathsensor.cpp 中定義，這裡只是備用)
float move_speed = 16.0f; // 路徑跟隨的速度
float original_move_speed = 16.0f; // 儲存原始速度
float max_speed = 0.5; // Maximum speed for the chassis

// 移動距離和方向
float travel_distance = 0; // Distance traveled since the last update
int dir = 0; // Direction for direct move, default is 0 (forward)
float path_dis = 0; // Distance to follow the path

// 線條類型和路徑方向
int line_type = 0;
int path_dir = 0; // Direction for path following, default is 0 (forward)

int start_x,start_y,start_theta; // Starting position and orientation for path following

extern float cmd_v_x,cmd_v_y,cmd_v_w;  // 改為 extern，在 chassis.cpp 中定義
extern float v_x, v_y, v_w;  // 實際速度（來自運動學轉換）

int move_dir;
int ach_check;
int stage;

// ============ 統一速度控制系統初始化函式 ============
void initSpeedController() {
    // 使用預設配置初始化速度控制器
    SpeedProfile default_profile;
    default_profile.max_speed = 50.0f;        // 16 units/s (根據原始程式碼調整)
    default_profile.acceleration = 12.0f;      // 8 units/s²
    default_profile.deceleration = 12.0f;     // 12 units/s²
    default_profile.jerk_limit = 20.0f;       // 20 units/s³
    default_profile.min_speed_ratio = 0.3f;   // 最小10%速度
    
    // 角速度專門配置 - 保守參數避免振盪
    default_profile.max_angular_speed = 1.2f;     // 0.8 rad/s (約 46度/秒)
    default_profile.angular_acceleration = 0.6f;  // 0.3 rad/s²
    default_profile.angular_deceleration = 0.8f;  // 0.8 rad/s²
    default_profile.angular_jerk_limit = 3.0f;    // 3 rad/s³
    default_profile.angular_speed_ratio = 0.4f;   // 10% 比例
    
    g_speed_controller.setProfile(default_profile);
    g_speed_controller.setSmoothEnabled(true);
    g_speed_controller.reset();
}

void configureSpeedProfile(float max_speed, float acceleration, float deceleration) {
    SpeedProfile profile = g_speed_controller.getProfile();
    profile.max_speed = max_speed;
    profile.acceleration = acceleration;
    profile.deceleration = (deceleration > 0) ? deceleration : acceleration * 1.5f;
    profile.jerk_limit = acceleration * 2.5f;  // 設為加速度的2.5倍
    
    g_speed_controller.setProfile(profile);
}

// 新增：專門的角速度配置函式
void configureAngularSpeedProfile(float max_angular_speed, float angular_acceleration, float angular_deceleration) {
    SpeedProfile profile = g_speed_controller.getProfile();
    profile.max_angular_speed = max_angular_speed;
    profile.angular_acceleration = angular_acceleration;
    profile.angular_deceleration = (angular_deceleration > 0) ? angular_deceleration : angular_acceleration * 1.5f;
    profile.angular_jerk_limit = angular_acceleration * 2.5f;  // 設為角加速度的2.5倍
    
    g_speed_controller.setProfile(profile);
}

void set_moveMode(int mode){
    moveMode_flag = mode;
}

bool achieve(){
    achieve_flag = true;
    return achieve_flag;
}

bool ach(){
    ach_check++;
    return achieve_flag;
}
bool missioncheck(){
    return mission_flag;
}


bool mission_complete(){
    mission_flag = true;
    return mission_flag;

}

bool arrive_destination(){
    if (fabs(goal_x - map_x) <  pos_threshold && fabs(goal_y - map_y) < pos_threshold) {
        if (fabs(goal_theta - theta) < theta_threshold) {
            return true;
        }
    }
    return false;
}

void stop(){
    // 使用統一速度控制器停止
    g_speed_controller.stop();
    g_speed_controller.updateSpeed();
    
    moveMode_flag = 0;
}

void moveMode(int mode) {
    switch (mode) {
        case 0: // stop
            stop();
            break;
        case 1: // move to a point without integral control
            integral_move_to(goal_x, goal_y, goal_theta * 180 / pi);//
            break;
        case 2: // follow path for a distance
            path_moveDis();
            break;
        case 3: // direct move till find line
            directMove_findLine();
            break;
        case 4: // move straight or rotate
        	direct_moveDistance();
            break;
        case 5:
            path_findLine();
            break;
        case 6:
            moveDistance(); // 新增：等速率移動功能
            break;
        default:
        	break;
    }
}

void integral_move(){
    if (arrive_destination()){
        g_speed_controller.stop();
        g_speed_controller.updateSpeed();
        moveMode_flag = 0;
        achieve_flag = true;
    }
    else{
        // Calculate the distance to the goal
        float dx = goal_x - map_x;
        float dy = goal_y - map_y;
        // float distance = sqrt(dx * dx + dy * dy); // 移除未使用的變數

        // Calculate the angle to the goal
        float dtheta = goal_theta - theta;
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        // 使用統一速度控制器的比例控制
        g_speed_controller.calculateProportionalSpeed(dx, dy, dtheta);
        g_speed_controller.updateSpeed();
        
        // 同時更新 Chassis::updateSpeed() 以保持相容性
        Chassis::updateSpeed();
    }
}

void followPath(int path_dir) { // follow path for mecanum chassis - 使用原來的控制方式
    // 直接調用原來的 weight 函數進行尋跡控制
    // 這樣可以避免 SpeedController 的加減速影響尋跡性能
    weight(path_dir);
}

void path_moveDis() {
    float dx = map_x - start_x;
    float dy = map_y - start_y;
    float travel_distance = sqrtf(dx * dx + dy * dy);
    
    if (travel_distance <= path_dis) {
        // 使用原來的尋跡控制方式，不套用 SpeedController 的加減速
        followPath(path_dir);
        return;
    }
    else {
        // 停止並標記完成
        stop(); // 使用原來的 stop() 函式
        achieve(); // Mark as achieved
    }
}

void direct_move(int direction, float speed) {
    // 設定基準速度（可從外部配置）
    float base_linear_speed = speed;  // 使用傳入的速度參數
    float base_angular_speed = speed * 0.1f; // 角速度為線速度的10%

    switch (direction) {
        case 0: // Move forward
            g_speed_controller.setTargetSpeed(0, base_linear_speed, 0);
            break;
        case 1: // Move backward
            g_speed_controller.setTargetSpeed(0, -base_linear_speed, 0);
            break;
        case 2: // Move right
            g_speed_controller.setTargetSpeed(base_linear_speed, 0, 0);
            break;
        case 3: // Move left
            g_speed_controller.setTargetSpeed(-base_linear_speed, 0, 0);
            break;
        case 4: // Rotate counterclockwise
            g_speed_controller.setTargetSpeed(0, 0, base_angular_speed);
            break;
        case 5: // Rotate clockwise
            g_speed_controller.setTargetSpeed(0, 0, -base_angular_speed);
            break;
        default:
            g_speed_controller.stop(); // Stop if the direction is invalid
    }
    
    // 更新速度控制器
    g_speed_controller.updateSpeed();
}

void directMove_findLine() {
    if (!line_check(line_type)) {
        // 使用統一的 direct_move 函式，並傳入速度參數
        direct_move(move_dir, normal_Speed);
    } else {
        g_speed_controller.stop();
        g_speed_controller.updateSpeed();
        achieve(); // Mark as achieved
    }
}

void direct_moveDistance() {
    //float velocity = 0.2; // Set a constant speed
    //float spin = 0.1;     // Set a constant spin rate
    float dx = map_x - start_x;
    float dy = map_y - start_y;
    float travel_distance = sqrtf(dx * dx + dy * dy);
    float dtheta = fabs(theta - start_theta);

    switch (move_dir) {
        case 0: // Move forward
            if (travel_distance <= distance) {
                direct_move(move_dir, direct_move_speed);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 1: // Move backward
            if (travel_distance <= distance) {
                direct_move(move_dir, direct_move_speed);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 2: // Move right
            if (travel_distance <= distance) {
                direct_move(move_dir, direct_move_speed);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 3: // Move left
            if (travel_distance <= distance) {
                direct_move(move_dir, direct_move_speed);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 4: // Rotate counterclockwise
            if (dtheta <= angle) {
                direct_move(dir, direct_move_speed);
            } else {
                stop();
                achieve();
            }
            break;
        case 5: // Rotate clockwise
            if (dtheta <= angle) {
                direct_move(dir, direct_move_speed);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        default:
            stop();
    }
}

void path_findLine() {
    // Check the type of line to find
    if (!line_check(line_type)) {
        followPath(path_dir); // 使用恢復原控制方式的 followPath
        return;       // If the line is not found, continue following the path
    } else {
        stop(); // 使用原來的 stop() 函式
        achieve();
    }
}

void direct_move_find_line() {
    if(!line_check(line_type)) {
        direct_move(move_dir, normal_Speed); // 使用統一的 direct_move 函式
    } else {
        g_speed_controller.stop();
        g_speed_controller.updateSpeed();
        achieve(); // Mark as achieved
    }
}


void moveDistance(){
    // 計算總移動距離
    dis = sqrt(x_distance * x_distance + y_distance * y_distance);
    // 計算已移動的距離
    float dx_traveled = map_x - start_x;
    float dy_traveled = map_y - start_y;
    float traveled_distance = sqrt(dx_traveled * dx_traveled + dy_traveled * dy_traveled);
    
    // 計算角度誤差 - 不進行正規化，保留 setmoveDistance 設定的路徑方向
    float angle_error = moveangle - theta;
    // 安全限制：防止角度差過大導致異常行為
    if (fabs(angle_error) > 4 * pi) {  // 超過 720° 就是異常了
        // 強制正規化到合理範圍
        while (angle_error > 2 * pi) angle_error -= 2 * pi;
        while (angle_error < -2 * pi) angle_error += 2 * pi;
    }
    
    // 設定速度控制器的參數
    g_speed_controller.setMaxSpeed(moveDistance_speed);
    g_speed_controller.setSmoothEnabled(accel);
    
    // 如果移動距離太小但需要旋轉，執行原地旋轉
    if (dis < pos_threshold) {
        if (fabs(angle_error) < 0.01f) {  // 使用更小的停止閾值 0.01 弧度 (~0.57 度)
            g_speed_controller.stop();
            g_speed_controller.updateSpeed();
            achieve();
            return;
        } else {
            // 純旋轉模式 - 直接計算角速度，不使用 SpeedController 的復雜計算
            float kp_pure_rotation = 0.8f; // 純旋轉的比例增益
            float angular_velocity = kp_pure_rotation * angle_error;
            
            // 限制角速度範圍
            float max_angular_speed = 1.0f; // 純旋轉的最大角速度
            if (fabs(angular_velocity) > max_angular_speed) {
                angular_velocity = (angular_velocity > 0) ? max_angular_speed : -max_angular_speed;
            }
            
            // 確保最小角速度（避免過慢）
            float min_angular_speed = 0.1f;
            if (fabs(angle_error) > 0.01f && fabs(angular_velocity) < min_angular_speed) {  // 調整最小速度條件
                angular_velocity = (angular_velocity > 0) ? min_angular_speed : -min_angular_speed;
            }
            
            // 直接設定角速度，不經過 SpeedController 的加減速處理
            g_speed_controller.setTargetSpeed(0, 0, angular_velocity);
            g_speed_controller.setSmoothEnabled(false); // 純旋轉不使用平滑控制
            g_speed_controller.updateSpeed();
            return;
        }
    }
    
    // 檢查是否已達到目標
    if (traveled_distance >= dis) {
        g_speed_controller.stop();
        g_speed_controller.updateSpeed();
        achieve();
        return;
    }
    
    // 計算剩餘需要移動的距離
    float remaining_x = x_distance - dx_traveled;
    float remaining_y = y_distance - dy_traveled;
    float remaining_distance = sqrt(remaining_x * remaining_x + remaining_y * remaining_y);
    
    // 如果剩餘距離太小，直接完成
    if (remaining_distance < pos_threshold) {
        g_speed_controller.stop();
        g_speed_controller.updateSpeed();
        achieve();
        return;
    }
    
    // 計算目標移動方向
    float target_angle = atan2(remaining_y, remaining_x);
    while (target_angle > M_PI) target_angle -= 2 * M_PI;
    while (target_angle < -M_PI) target_angle += 2 * M_PI;
    
    // 使用統一控制器的軌跡速度計算
    if (accel) {
        g_speed_controller.calculateTrajectorySpeed(remaining_distance, dis);
    }
    
    // 計算基準速度分量
    float vx_global = moveDistance_speed * cos(target_angle);
    float vy_global = moveDistance_speed * sin(target_angle);
    
    // 轉換到機器人本體座標系
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float vx_robot = cos_theta * vx_global + sin_theta * vy_global;
    float vy_robot = -sin_theta * vx_global + cos_theta * vy_global;
    
    // 角度控制 - 如果需要同時旋轉
    float angular_velocity = 0.0f;
    if (fabs(angle_error) > theta_threshold) {
        // 簡化的角度控制，避免復雜的加減速計算
        float kp_angular = 0.8f; // 降低比例係數，減少震盪
        angular_velocity = kp_angular * angle_error;
        
        // 限制角速度範圍
        float max_angular_speed = 0.6f; // 降低最大角速度
        if (fabs(angular_velocity) > max_angular_speed) {
            angular_velocity = (angular_velocity > 0) ? max_angular_speed : -max_angular_speed;
        }
        
        // 最小角速度限制
        float min_angular_speed = 0.05f;
        if (fabs(angular_velocity) < min_angular_speed) {
            angular_velocity = (angular_velocity > 0) ? min_angular_speed : -min_angular_speed;
        }
    }
    
    // 設定目標速度
    g_speed_controller.setTargetSpeed(vx_robot, vy_robot, angular_velocity);
    
    // 對於同時移動和旋轉的情況，啟用平滑控制
    if (dis >= pos_threshold) {
        g_speed_controller.setSmoothEnabled(accel);
    }
    
    g_speed_controller.updateSpeed();
}

//use in main function
void integral_move_to (float x,float y,float w){
    goal_x = x;
    goal_y = y;
    goal_theta = w * pi / 180; // Convert to radians
    moveMode_flag = 1;
    achieve_flag = false;
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;
}

void setPath_distance(float path_distance, int _path_dir, float speed) {
    path_dis = path_distance;
    path_dir = _path_dir; // Set the direction for path following
    moveMode_flag = 2;
    achieve_flag = false;
    travel_distance = 0; // Reset travel distance for the new path
    start_x = map_x; // Store the starting position
    start_y = map_y; // Store the starting position
    
    // 設定循跡速度`
    normal_Speed = speed;
    // 儲存當前的 move_speed 作為原始速度
    original_move_speed = normal_Speed;
    
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;

}

void set_directMove_findLine(int _dir, int _line_type, float speed) {
    moveMode_flag = 3; // Set the mode to follow line
    achieve_flag = false; // Reset the achievement flag
    line_type = _line_type; // Set the line type
    move_dir = _dir; // Set the direction for direct move

    // Set the speed for direct move
    normal_Speed = speed;

    while (!ach()) {
        ROS1::spinCycle();
    }
   // ach_stage++;
}


void set_directMove(int direction, float _distance, float _angle, float speed) {//move sidtance
    move_dir = direction;
    start_x = map_x; // Store the starting position
    start_y = map_y; // Store the starting position
    start_theta = theta; // Store the starting orientation
    distance = _distance; // Set the distance to move
    angle = _angle; // Set the angle to rotate
    direct_move_speed = speed; // Set the speed for direct move
    moveMode_flag = 4; // Set the mode to direct move
    achieve_flag = false; // Reset the achievement flag
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;
}

void setPath_finding_line(int _line_type, int _path_dir, float speed) {
    moveMode_flag = 5; // Set the mode to finding line
    line_type = _line_type; // Set the line type
    path_dir = _path_dir; // Default direction for path following
    achieve_flag = false; // Reset the achievement flag
    
    // 設定循跡速度
    normal_Speed = speed;
    // 儲存當前的 move_speed 作為原始速度
    original_move_speed = normal_Speed;
    
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;
}
//int currentStage = 0;
void waitMissionComplete(int _stage){
	if(ttest)
		return;
    stage = _stage;
    moveMode_flag = 0;
    mission_flag = false; // 修正：使用賦值運算符而非比較運算符
    //currentStage = _stage;
    while(!missioncheck()) {
        ROS1::spinCycle();
    }
    stage = 0 ;
}

// ============ 尋跡參數調整函式實作 ============
// 這些函式允許在運行時調整尋跡的加減速參數

// 在 pathsensor.cpp 中定義的外部變數
extern float pathfinding_speed_change_rate;  // 對應 SPEED_CHANGE_RATE
extern float pathfinding_min_speed_ratio;    // 對應 MIN_SPEED_RATIO  
extern int pathfinding_line_threshold;       // 對應 LINE_LOST_THRESHOLD

void setPathfindingAcceleration(float speed_change_rate) {
    if (speed_change_rate > 0.1f && speed_change_rate <= 10.0f) {
        pathfinding_speed_change_rate = speed_change_rate;
    }
}

void setPathfindingMinSpeed(float min_speed_ratio) {
    if (min_speed_ratio >= 0.1f && min_speed_ratio <= 0.8f) {
        pathfinding_min_speed_ratio = min_speed_ratio;
    }
}

void setPathfindingThreshold(int line_lost_threshold) {
    if (line_lost_threshold >= 500 && line_lost_threshold <= 2000) {
        pathfinding_line_threshold = line_lost_threshold;
    }
}

void getPathfindingParams(float* speed_rate, float* min_ratio, int* threshold) {
    if (speed_rate) *speed_rate = pathfinding_speed_change_rate;
    if (min_ratio) *min_ratio = pathfinding_min_speed_ratio;
    if (threshold) *threshold = pathfinding_line_threshold;
}

void setmoveDistance(float x_dis, float y_dis ,float angle ,float speed,int _accel, int rotation_dir ){
    moveMode_flag = 6;
    x_distance = x_dis;
    y_distance = y_dis;
    
    // 根據旋轉方向控制處理角度
    // rotation_dir: 0=自動選擇最短路徑, 1=順時針, -1=逆時針, 2=絕對角度
    if (rotation_dir == 2) {
        // 絕對角度模式：直接使用給定角度
        moveangle = angle * pi / 180; // Convert degrees to radians
        // 正規化到 [-π, π] 以避免多轉問題
        while (moveangle > pi) moveangle -= 2 * pi;
        while (moveangle < -pi) moveangle += 2 * pi;
    } else if (fabs(angle) < 0.1f) {
        // 如果角度接近 0，不進行旋轉，保持當前角度
        moveangle = theta; // 保持當前角度，不旋轉
    } else {
        // 相對角度模式：基於當前角度計算目標角度
        float target_angle_rad = angle * pi / 180;      // 目標角度轉為弧度
        float current_angle_rad = theta;                // 當前角度已經是弧度
        
        // 計算角度差（弧度）
        float angle_diff = target_angle_rad - current_angle_rad;
        
        // 正規化角度差到 [-π, π] 弧度
        while (angle_diff > pi) angle_diff -= 2 * pi;
        while (angle_diff < -pi) angle_diff += 2 * pi;
        
        if (rotation_dir == 0) {
            // 自動選擇最短路徑（預設行為）
            moveangle = current_angle_rad + angle_diff;
        } else if (rotation_dir == 1) {
            // 強制順時針旋轉
            // 在你的座標系中，順時針 = 角度值減少 = 負角速度
            if (angle_diff >= 0) {
                // 如果自然方向是逆時針或零，強制走順時針路徑
                angle_diff -= 2 * pi;
            }
            moveangle = current_angle_rad + angle_diff;
        } else if (rotation_dir == -1) {
            // 強制逆時針旋轉  
            // 在你的座標系中，逆時針 = 角度值增加 = 正角速度
            if (angle_diff <= 0) {
                // 如果自然方向是順時針或零，強制走逆時針路徑
                angle_diff += 2 * pi;
            }
            moveangle = current_angle_rad + angle_diff;
        }

        // 不進行最終角度正規化！讓 moveangle 可以超出 [-π, π] 範圍
        // 這樣 moveDistance() 才能知道要走長路徑還是短路徑
    }
    
    moveDistance_speed = speed;
    
    // 計算當前的實際移動速度作為初始速度（基於encoder反饋）
    initial_speed = sqrt(v_x * v_x + v_y * v_y);
    
    // 如果當前速度為0，設定一個最小初始速度
    if (initial_speed < 0.1f) {
        initial_speed = speed * 0.1f; // 設為目標速度的10%
    }
    
    // 確保初始速度不超過目標速度
    if (initial_speed > speed) {
        initial_speed = speed * 0.8f; // 如果當前速度太高，設為目標速度的80%
    }
    
    current_move_speed = initial_speed; // 初始化當前速度
    
    // 初始化起始位置
    start_x = map_x;
    start_y = map_y;
    start_theta = theta;
    accel = _accel;
    
    // 重置成就標誌
    achieve_flag = false;
    
    // 計算總距離
    dis = sqrt(x_distance * x_distance + y_distance * y_distance);
    
    // 設定加速/減速距離為總距離的10%
    acceleration_distance = dis * 0.1f;
    
    // 等待執行完成
    while (!ach()) {
    	ROS1::spinCycle();
    }
    //ach_stage++;
}

//void setLifter(int frontLifter,int backLifter,float chassisSP){
//    moveMode_flag = 6;
//    Lifter::goalHeights[0] = frontLifter;
//    Lifter::goalHeights[1] = backLifter;
//    Lifter::wheel_sp = chassisSP;
//    achieve_flag = false;
//    while(!ach()) {}
//}

void chassis_move(){
	moveMode(moveMode_flag);
}

void updatePosition(float x, float y, float z){
	map_x = x;
	map_y = y;
	theta = z / (180 / pi);  // Convert degrees to radians
	// 正規化 theta 到 [-π, π] 以避免累積錯誤和晃動問題
	while (theta > pi) theta -= 2 * pi;
	while (theta < -pi) theta += 2 * pi;
}





