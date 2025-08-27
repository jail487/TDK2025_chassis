/*
 * chassis_move.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_CHASSIS_MOVE_H_
#define INC_CHASSIS_MOVE_H_

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
void path_moveDis(float path_distance,int _path_dir); //2
void direct_move(int direction);
void directMove_findLine(int _dir, int _line_type);
void direct_moveDistance(int direction);
void path_findLine(int line_type,int _path_dir);
void direct_move_find_line(int line_type, int direction);

//update move mode
void integral_move_to (float x,float y,float w);
void setPath_distance(float path_distance,int _path_dir);
void set_directMove_findLine(int _line_type, int _dir);
void set_directMove(int direction, float _distance, float _angle);
void setPath_finding_line(int _line_type, int _path_dir);

void chassis_move();








#endif /* INC_LOCATION_H_ */
