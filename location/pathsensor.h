/*
 * pathSensor.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#ifndef INC_PATHSENSOR_H_
#define INC_PATHSENSOR_H_

#include "stm32h7xx_hal.h"
#include <math.h>

void path_setup();
void path(int dir);

//go to (x,y)
void path_moveto(float path_d);

void weight(int dir);
void smooth_speed_update(float target_x, float target_y);
bool check_line_lost(int dir);

//1:front find line, 2:middle find line, 3:find cross road, 4:find line
bool line_check(int type);

#endif /* INC_PATHSENSOR_H_ */

