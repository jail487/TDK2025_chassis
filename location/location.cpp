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
//#include "motor.h"
#include "pathSensor.h"
#include "chassis.h"
#include "stdlib.h"
#include "cmath"
#include <math.h>

#define pi 3.14159

extern float path_dis, path_motor_speed[2];
extern float map_x,map_y,theta;
extern uint32_t adcRead[7];

float goal_x = 0, goal_y = 0, goal_theta = 0;
float last_x = 0, last_y = 0, last_theta = 0;

int move_mode_flag = 0; // 0: move to a point, 1: move straight, 2: follow path for a distance, 3: follow path
bool achieve_flag = false;
bool mission_flag = false;

float pos_threshold = 0.1; // Threshold for position accuracy
const float theta_threshold = 0.1; // Threshold for orientation accuracy

const float kp_pos = 0.5;  // Proportional gain for position control
const float kp_theta = 20; // Proportional gain for orientation control

float max_speed = 0.5; // Maximum speed for the chassis

float travel_distance = 0; // Distance traveled since the last update

int dir = 0; // Direction for direct move, default is 0 (forward)

float path_dis = 0; // Distance to follow the path

int line_type = 0;
int path_dir = 0; // Direction for path following, default is 0 (forward)

int start_x,start_y,start_theta; // Starting position and orientation for path following

int cmd_v_x,cmd_v_y,cmd_v_w;

int move_dir;
int ach_check;
void set_move_mode(int mode){
    move_mode_flag = mode;
}

bool achieve(){
    achieve_flag = true;
    return achieve_flag;
}

bool ach(){
    ach_check++;
    return achieve_flag;
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
    chassis_update_speed(0, 0, 0);
}

void move_mode(int mode) {
    switch (mode) {
        case 0: // stop
            stop();
            break;
        case 1: // move to a point without integral control
            integral_move_to(goal_x, goal_y, goal_theta * 180 / pi);//
            break;
        case 2: // follow path for a distance
            path_moveDis(path_dis, path_dir);
            break;
        case 3: // direct move till find line
            directMove_findLine(move_dir,line_type);
            break;
        case 4: // move straight or rotate
            direct_move(dir);
            break;
        case 5:
            path_findLine(line_type,path_dir);
            break;
        case 6:

        	break;
    }
}

void integral_move(){
    if (arrive_destination()){
        move_mode_flag = 0;
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
        chassis_update_speed(chassis_speed_x, chassis_speed_y, speed_w);


        // Move towards the goal
        chassis_update_speed(distance * cos(required_theta), distance * sin(required_theta), required_theta);
    }
}

void followPath(int path_dir) { // follow path for mecanum chassis
        weight(path_dir); // sets cmd_v_y, cmd_v_x, cmd_v_w
}

void path_moveDis(float path_distance,int _path_dir) {
    path_dir = _path_dir; // Set the direction for path following
    float dx = map_x - start_x;
    float dy = map_y - start_y;
    float travel_distance = sqrtf(dx * dx + dy * dy);
    if (travel_distance <= path_distance) {
        followPath(_path_dir); // Follow the path
        travel_distance += sqrtf(dx * dx + dy * dy); // Fixed incomplete statement
        return;
    }
    else {
        // Stop the chassis when the path distance is reached
        stop();
        achieve(); // Mark as achieved
    }
}

void direct_move(int direction) {
    switch (direction) {
        case 0: // Move forward
            cmd_v_x = 0;
            cmd_v_y = 0.2; // Set a constant speed
            cmd_v_w = 0;
            break;
        case 1: // Move backward
            cmd_v_x = 0;
            cmd_v_y = -0.2; // Set a constant speed
            cmd_v_w = 0;
            break;
        case 2: // Move right
            cmd_v_x = 0.2; // Set a constant speed
            cmd_v_y = 0;
            cmd_v_w = 0;
            break;
        case 3: // Move left
            cmd_v_x = -0.2; // Set a constant speed
            cmd_v_y = 0;
            cmd_v_w = 0;
            break;
        case 4: // Rotate counterclockwise
            cmd_v_x = 0;
            cmd_v_y = 0;
            cmd_v_w = 0.1; // Set a constant spin rate
            break;
        case 5: // Rotate clockwise
            cmd_v_x = 0;
            cmd_v_y = 0;
            cmd_v_w = -0.1; // Set a constant spin rate
            break;
        default:
            stop(); // Stop if the direction is invalid
    }
}
void directMove_findLine(int _dir, int _line_type) {
    if (!line_check(_line_type)) {
        direct_move(_dir); // Move in the specified direction
    } else {
        stop(); // Stop if the line is found
        achieve(); // Mark as achieved
    }
}

float distance = 0.f; // Distance to move in direct_move
float angle = 0.f; // Angle to rotate in direct_move
void direct_moveDistance(int direction) {
    //float velocity = 0.2; // Set a constant speed
    //float spin = 0.1;     // Set a constant spin rate
    float dx = map_x - start_x;
    float dy = map_y - start_y;
    float travel_distance = sqrtf(dx * dx + dy * dy);
    float dtheta = fabs(theta - start_theta);

    switch (direction) {
        case 0: // Move forward
            if (travel_distance <= distance) {
                direct_move(dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 1: // Move backward
            if (travel_distance <= distance) {
                direct_move(dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 2: // Move right
            if (travel_distance <= distance) {
                direct_move(dir);
                return;
            } else {
                stop();
                achieve();
            }
            break;
        case 3: // Move left
            if (travel_distance <= distance) {
                direct_move(dir);
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

void path_findLine(int line_type,int _path_dir) {
    // Check the type of line to find
    if (!line_check(line_type)) {
        followPath(_path_dir);
        return;       // If the line is not found, continue following the path
    } else {
        //stop();
        achieve();
    }
}

void direct_move_find_line(int line_type, int direction) {

    if(!line_check(line_type)) {
        direct_move(direction); // Move in the specified direction
    } else {
        stop(); // Stop if the line is found
        achieve(); // Mark as achieved
    }
}

//use in main function
void integral_move_to (float x,float y,float w){
    goal_x = x;
    goal_y = y;
    goal_theta = w * pi / 180; // Convert to radians
    move_mode_flag = 1;
    achieve_flag = false;
    while (!ach()) {}
}

void setPath_distance(float path_distance,int _path_dir) {
    path_dis = path_distance;
    path_dir = _path_dir; // Set the direction for path following
    move_mode_flag = 2;
    achieve_flag = false;
    travel_distance = 0; // Reset travel distance for the new path
    start_x = map_x; // Store the starting position
    start_y = map_y; // Store the starting position
    while (!ach()) {}
}

void set_directMove_findLine(int _dir, int _line_type) {
    move_mode_flag = 3; // Set the mode to follow line
    achieve_flag = false; // Reset the achievement flag
    line_type = _line_type; // Set the line type
    dir = _dir; // Set the direction for direct move
    while (!ach()) {}
}


void set_directMove(int direction, float _distance, float _angle) {
    dir = direction;
    start_x = map_x; // Store the starting position
    start_y = map_y; // Store the starting position
    start_theta = theta; // Store the starting orientation
    distance = _distance; // Set the distance to move
    angle = _angle; // Set the angle to rotate
    move_mode_flag = 4; // Set the mode to direct move
    achieve_flag = false; // Reset the achievement flag
    while (!ach()) {}
}

void setPath_finding_line(int _line_type, int _path_dir) {
    move_mode_flag = 5; // Set the mode to finding line
    line_type = _line_type; // Set the line type
    path_dir = _path_dir; // Default direction for path following
    achieve_flag = false; // Reset the achievement flag
    while (!ach()) {}
}

void chassis_move(){
	move_mode(move_mode_flag);
}






