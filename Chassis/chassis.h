/*
 * chassis.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_


namespace Chassis {

void setup();
void mecan_IK_transform(float _v_x,float _v_y,float _v_w);
void mecan_FK_transform();
void localization();
void updateSpeed(float _v_x,float _v_y,float _v_w);
void chassis_task();

}









#endif /* CHASSIS_H_ */
