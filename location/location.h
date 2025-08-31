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

//update move mode
void integral_move_to (float x,float y,float w);
void setPath_distance(float path_distance,int _path_dir);
void set_directMove_findLine(int _line_type, int _dir);
void set_directMove(int direction, float _distance, float _angle);
void setPath_finding_line(int _line_type, int _path_dir);
void setLifter(int frontLifter,int backLifter,float chassisSP);


void chassis_move();






#endif /* INC_LOCATION_H_ */
