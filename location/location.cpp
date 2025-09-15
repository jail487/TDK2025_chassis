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


#define pi 3.14159265358979323846
extern int ach_stage;
extern float path_dis, path_motor_speed[2];
//extern int mission_complete ;
//// direct_move 相關變數
float distance = 0.f; // Distance to move in direct_move
float angle = 0.f; // Angle to rotate in direct_move

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
const float theta_threshold = 0.001; // Threshold for orientation accuracy
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
    // 使用 chassis.cpp 的統一接口來停止
    Chassis::setSpeed(0, 0, 0);

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
        moveMode_flag = 0;
        achieve_flag = true;
    }
    else{
        // Calculate the distance to the goal
        float dx = goal_x - map_x;
        float dy = goal_y - map_y;
        float distance = sqrt(dx * dx + dy * dy);

        // Calculate the angle to the goal
        float dtheta = goal_theta - theta;
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        float required_theta = atan2(dy, dx); // Angle to the goal in radians
        while (required_theta > M_PI) required_theta -= 2 * M_PI;
        while (required_theta < -M_PI) required_theta += 2 * M_PI;
        // Calculate the speed based on position error
        float speed_x = kp_pos * dx;
        float speed_y = kp_pos * dy;
        float speed_w = kp_theta * dtheta;
        // Limit the speed to a maximum value
        float speed_norm = sqrt(speed_x * speed_x + speed_y * speed_y);
        if (speed_norm > max_speed) {
            speed_x = (speed_x / speed_norm) * max_speed;
            speed_y = (speed_y / speed_norm) * max_speed;
        }
        if (fabs(speed_w) > max_speed) {
            speed_w = (speed_w > 0 ? max_speed : -max_speed);
        }
        //transform the speed to the chassis frame
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);
        float chassis_speed_x = cos_theta * speed_x - sin_theta * speed_y;
        float chassis_speed_y = sin_theta * speed_x + cos_theta * speed_y;
        // Update the chassis speed
        Chassis::setSpeed(chassis_speed_x, chassis_speed_y, speed_w);
        Chassis::updateSpeed();


        // Move towards the goal
        Chassis::setSpeed(distance * cos(required_theta), distance * sin(required_theta), required_theta);
        Chassis::updateSpeed();
    }
}

void followPath(int path_dir) { // follow path for mecanum chassis with integrated weight function
    // 需要的變數定義
    extern uint16_t adcRead_ADC3[12];  // ADC 讀取陣列
    extern float weight_err, weight_lasttime, weight_change;
    
    // 常數定義
    const float LINE_LOST_THRESHOLD = 1000.0f;  // 所有傳感器數值低於此閾值時認為失去線條
    const float MIN_SPEED_RATIO = 0.6f;         // 提高最小速度比例到60%，減少速度變化
    const float w_kp_y = 0.4f;
    const float w_kp_x = 0.15f;
    const float w_kd = 0.0f;
    
    // 檢測是否失去線條
    bool line_lost = false;
    if (path_dir == 0) { // front
        line_lost = (adcRead_ADC3[0] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[1] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[2] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[3] < LINE_LOST_THRESHOLD);
    }
    else if (path_dir == 1) { // back
        line_lost = (adcRead_ADC3[0] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[1] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[3] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[4] < LINE_LOST_THRESHOLD);
    }
    else if (path_dir == 2) { // right
        line_lost = (adcRead_ADC3[8] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[9] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[10] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[11] < LINE_LOST_THRESHOLD);
    }
    else if (path_dir == 3) { // left
        line_lost = (adcRead_ADC3[4] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[5] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[6] < LINE_LOST_THRESHOLD && 
                    adcRead_ADC3[7] < LINE_LOST_THRESHOLD);
    }
    
    // 如果失去線條，降低速度繼續前進
    if (line_lost) {
        float min_speed = move_speed * MIN_SPEED_RATIO;
        
        if (path_dir == 0) {        // front
            Chassis::setSpeed(0, min_speed, 0);
        }
        else if (path_dir == 1) {   // back
            Chassis::setSpeed(0, -min_speed, 0);
        }
        else if (path_dir == 2) {   // right
            Chassis::setSpeed(min_speed, 0, 0);
        }
        else if (path_dir == 3) {   // left
            Chassis::setSpeed(-min_speed, 0, 0);
        }
        return;
    }
    
    // Calculate line following error (P and D)
    if (path_dir == 0){//front
        weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[2] + 3*adcRead_ADC3[3]) /
                     (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[2] + adcRead_ADC3[3]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        Chassis::setSpeed(0, move_speed, weight_err * w_kp_y + weight_change * w_kd);
    }
    else if(path_dir == 1){//back
        weight_err = ((float)(-3*adcRead_ADC3[0] - adcRead_ADC3[1] + adcRead_ADC3[3] + 3*adcRead_ADC3[4]) /
                    (adcRead_ADC3[0] + adcRead_ADC3[1]+ 4096 +adcRead_ADC3[3] + adcRead_ADC3[4]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        Chassis::setSpeed(0, -move_speed, weight_err * w_kp_y + weight_change * w_kd);
    }
    else if(path_dir == 2){//right
        weight_err = ((float)(-3*adcRead_ADC3[8] - adcRead_ADC3[9] + adcRead_ADC3[10] + 3*adcRead_ADC3[11]) /
                           (adcRead_ADC3[8] + adcRead_ADC3[9]+ 4096 +adcRead_ADC3[10] + adcRead_ADC3[11]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        Chassis::setSpeed(move_speed, 0, weight_err * w_kp_x + weight_change * w_kd);
    }
    else if(path_dir == 3){//left
        weight_err = ((float)(-3*adcRead_ADC3[4] - adcRead_ADC3[5] + adcRead_ADC3[6] + 3*adcRead_ADC3[7]) /
                         (adcRead_ADC3[4] + adcRead_ADC3[5]+ 4096 +adcRead_ADC3[6] + adcRead_ADC3[7]));
        weight_change = weight_err - weight_lasttime;
        weight_lasttime = weight_err;

        Chassis::setSpeed(-move_speed, 0, weight_err * w_kp_x + weight_change * w_kd);
    }
}

void path_moveDis() {
    float dx = map_x - start_x;
    float dy = map_y - start_y;
    float travel_distance = sqrtf(dx * dx + dy * dy);
    
    if (travel_distance <= path_dis) {
        // 平滑的速度控制，避免劇烈變化
        float speed_scale = 1.0f;
        float remaining_distance = path_dis - travel_distance;
        float decel_distance = path_dis * 0.4f; // 擴大減速區間到40%，更溫和
        
        // 使用更平滑的減速曲線
        if (remaining_distance < decel_distance) {
            // 使用平方根函數創造更平滑的減速曲線
            float progress = remaining_distance / decel_distance;
            float smooth_progress = sqrt(progress); // 平方根曲線比線性更平滑
            speed_scale = 0.5f + smooth_progress * 0.5f; // 從50%到100%，減少速度變化幅度
        }
        
        // 調整 move_speed
        move_speed = original_move_speed * speed_scale;
        
        // 確保最小速度不會太低，避免劇烈變化
        if (move_speed < 5) {  // 提高最小速度從2提高到5
            move_speed = 5.0f;
        }
        
        followPath(path_dir); // Follow the path
        return;
    }
    else {
        // 恢復原始速度並停止
        move_speed = original_move_speed;
        stop();
        achieve(); // Mark as achieved
    }
}

void direct_move(int direction) {
    switch (direction) {
        case 0: // Move forward
            Chassis::setSpeed(0, 16, 0);
            break;
        case 1: // Move backward
            Chassis::setSpeed(0, -16, 0);
            break;
        case 2: // Move right
            Chassis::setSpeed(16, 0, 0); // 修正：0.16 應該是 16
            break;
        case 3: // Move left
            Chassis::setSpeed(-16, 0, 0);
            break;
        case 4: // Rotate counterclockwise
            Chassis::setSpeed(0, 0, 0.1);
            break;
        case 5: // Rotate clockwise
            Chassis::setSpeed(0, 0, -0.1);
            break;
        default:
            stop(); // Stop if the direction is invalid
    }
}

void directMove_findLine() {
    if (!line_check(line_type)) {
        // 直接使用 chassis.cpp 的統一接口
        switch (move_dir) {
            case 0: // Move forward
                Chassis::setSpeed(0, 16, 0);
                break;
            case 1: // Move backward
                Chassis::setSpeed(0, -16, 0);
                break;
            case 2: // Move right
                Chassis::setSpeed(16, 0, 0);
                break;
            case 3: // Move left
                Chassis::setSpeed(-16, 0, 0);
                break;
            case 4: // Rotate counterclockwise
                Chassis::setSpeed(0, 0, 0.1);
                break;
            case 5: // Rotate clockwise
                Chassis::setSpeed(0, 0, -0.1);
                break;
            default:
                Chassis::setSpeed(0, 0, 0); // Stop if direction is invalid
        }
    } else {
        stop(); // Stop if the line is found
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
                direct_move(move_dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 1: // Move backward
            if (travel_distance <= distance) {
                direct_move(move_dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 2: // Move right
            if (travel_distance <= distance) {
                direct_move(move_dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 3: // Move left
            if (travel_distance <= distance) {
                direct_move(move_dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 4: // Rotate counterclockwise
            if (dtheta <= angle) {
                direct_move(dir);
            } else {
                stop();
                achieve();
            }
            break;
        case 5: // Rotate clockwise
            if (dtheta <= angle) {
                direct_move(dir);
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
        followPath(path_dir); // 使用已經整合統一速度控制的 followPath
        return;       // If the line is not found, continue following the path
    } else {
        //stop();
        achieve();
    }
}

void direct_move_find_line() {

    if(!line_check(line_type)) {
        direct_move(move_dir); // Move in the specified direction
    } else {
        stop(); // Stop if the line is found
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
    // 計算角度誤差
    float angle_error = moveangle - theta;
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;
    
    // 如果移動距離太小但需要旋轉，執行原地旋轉直到角度誤差小於閾值
    if (dis < pos_threshold) {
        if (fabs(angle_error) < theta_threshold) {
            stop();
            achieve();
            return;
        } else {
            // 只旋轉，不移動 - 使用改進的角速度控制
            float angle_traveled = fabs(theta - start_theta);
            float total_angle_needed = fabs(moveangle - start_theta);
            
            // 對於純旋轉，使用基於時間的角速度計算而非PID
            float synchronized_angular_velocity = 0.0f;
            
            // 估算完成剩餘角度所需的合理時間（秒）
            float desired_rotation_time = fabs(angle_error) / 1.5f; // 1.5 rad/s 作為基準角速度
            
            // 確保旋轉時間在合理範圍內
            desired_rotation_time = fmax(desired_rotation_time, 0.2f); // 最少 0.2 秒
            desired_rotation_time = fmin(desired_rotation_time, 3.0f); // 最多 3 秒
            
            // 計算同步角速度
            synchronized_angular_velocity = angle_error / desired_rotation_time;
            
            // 平滑的旋轉加減速
            if (accel) {
                float angle_progress = angle_traveled / total_angle_needed;
                if (angle_progress < 0.3f) { // 加速階段，擴大到30%
                    float smooth_accel = sqrt(angle_progress / 0.3f);
                    float scale = 0.5f + smooth_accel * 0.5f; // 從50%到100%
                    synchronized_angular_velocity *= scale;
                } else if (angle_progress > 0.7f) { // 減速階段，從70%開始
                    float remaining_progress = (1.0f - angle_progress) / 0.3f;
                    float smooth_decel = sqrt(remaining_progress);
                    float scale = 0.5f + smooth_decel * 0.5f;
                    synchronized_angular_velocity *= scale;
                }
            }
            
            // 限制角速度在安全範圍內
            if (fabs(synchronized_angular_velocity) > max_speed) {
                synchronized_angular_velocity = (synchronized_angular_velocity > 0) ? 
                    max_speed : -max_speed;
            }
            // 確保最小有效角速度（對於純旋轉更重要）
            float min_pure_rotation_velocity = 0.05f; // 純旋轉的最小角速度
            if (fabs(angle_error) > theta_threshold && fabs(synchronized_angular_velocity) < min_pure_rotation_velocity) {
                synchronized_angular_velocity = (angle_error > 0) ? 
                    min_pure_rotation_velocity : -min_pure_rotation_velocity;
            }
            
            Chassis::setSpeed(0, 0, synchronized_angular_velocity);
            return;
        }
    }
    
    // 檢查是否已達到目標
    if (traveled_distance >= dis) {
        stop();
        achieve();
        return;
    }
    
    // 計算剩餘需要移動的距離
    float remaining_x = x_distance - dx_traveled;
    float remaining_y = y_distance - dy_traveled;
    float remaining_distance = sqrt(remaining_x * remaining_x + remaining_y * remaining_y);
    
    // 如果剩餘距離太小，直接完成
    if (remaining_distance < pos_threshold) {
        stop();
        achieve();
        return;
    }
    
    // 動態計算移動方向（根據剩餘距離）
    float target_angle = atan2(remaining_y, remaining_x);
    
    // 正規化角度到 [-π, π]
    while (target_angle > M_PI) target_angle -= 2 * M_PI;
    while (target_angle < -M_PI) target_angle += 2 * M_PI;
    
    // 平滑的速度縮放計算
    float speed_scale = 1.0f;
    if (accel) {
        float remaining_ratio = remaining_distance / dis;
        if (remaining_ratio < 0.4f) { // 擴大減速階段到40%
            // 使用更平滑的減速曲線
            float smooth_progress = sqrt(remaining_ratio / 0.4f);
            speed_scale = 0.6f + smooth_progress * 0.4f; // 從60%到100%，減少變化幅度
        }
    }
    
    // 計算等速率移動的速度分量
    float vx_global = moveDistance_speed * cos(target_angle) * speed_scale;
    float vy_global = moveDistance_speed * sin(target_angle) * speed_scale;
    
    // 簡化角速度計算
    float synchronized_angular_velocity = 0.0f;
    if (fabs(angle_error) > theta_threshold) {
        synchronized_angular_velocity = kp_theta * angle_error * 0.1f; // 簡化的PID控制
        
        // 限制角速度
        if (fabs(synchronized_angular_velocity) > max_speed) {
            synchronized_angular_velocity = (synchronized_angular_velocity > 0) ? 
                max_speed : -max_speed;
        }
    }
    
    // 轉換到機器人本體座標系
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float vx_robot = cos_theta * vx_global + sin_theta * vy_global;
    float vy_robot = -sin_theta * vx_global + cos_theta * vy_global;
    
    // 直接使用 chassis.cpp 的統一接口
    Chassis::setSpeed(vx_robot, vy_robot, synchronized_angular_velocity);
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
    move_speed = speed;
    // 儲存當前的 move_speed 作為原始速度
    original_move_speed = move_speed;
    
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;

}

void set_directMove_findLine(int _dir, int _line_type) {
    moveMode_flag = 3; // Set the mode to follow line
    achieve_flag = false; // Reset the achievement flag
    line_type = _line_type; // Set the line type
    move_dir = _dir; // Set the direction for direct move
    
    while (!ach()) {
        ROS1::spinCycle();
    }
   // ach_stage++;
}


void set_directMove(int direction, float _distance, float _angle) {//move sidtance
    move_dir = direction;
    start_x = map_x; // Store the starting position
    start_y = map_y; // Store the starting position
    start_theta = theta; // Store the starting orientation
    distance = _distance; // Set the distance to move
    angle = _angle; // Set the angle to rotate
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
    move_speed = speed;
    // 儲存當前的 move_speed 作為原始速度
    original_move_speed = move_speed;
    
    while (!ach()) {
        ROS1::spinCycle();
    }
    //ach_stage++;
}
void waitMissionComplete(int _stage){
    stage = _stage; 
    moveMode_flag = 0;
    mission_flag == false;
    while(!missioncheck()) {
        ROS1::spinCycle();
    }
    //ach_stage++;
}

void setmoveDistance(float x_dis, float y_dis ,float angle ,float speed,int _accel){
    moveMode_flag = 6;
    x_distance = x_dis;
    y_distance = y_dis;
    
    // angle 參數表示絕對目標角度（度）
    moveangle = angle * pi / 180; // Convert degrees to radians
    
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






