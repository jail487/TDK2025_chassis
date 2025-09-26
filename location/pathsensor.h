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

void weight(int dir);////0:front,1:back,2:right,3:left
void smooth_speed_update(float target_x, float target_y);
bool check_line_lost(int dir);

//1:front find line, 2:middle find line, 3:find cross road, 4:find line
bool line_check(int type);
void setLinePI(float kp_y, float kp_x, float kd);

// ============ 尋跡參數調整變數 ============
// 這些變數替代原來的 #define，允許運行時調整
extern float pathfinding_speed_change_rate;  // 速度變化率 (替代 SPEED_CHANGE_RATE)
extern float pathfinding_min_speed_ratio;    // 最小速度比例 (替代 MIN_SPEED_RATIO)
extern int pathfinding_line_threshold;       // 失去線條閾值 (替代 LINE_LOST_THRESHOLD)

#endif /* INC_PATHSENSOR_H_ */

