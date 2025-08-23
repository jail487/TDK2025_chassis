/*
 * chassis.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_




void chassis_setup();
void mecan_IK_transform(float _v_x,float _v_y,float _v_w);
void mecan_FK_transform();
void update_speed(float _v_x,float _v_y,float _v_w);
void localization();
void chassis_update_speed(float _v_x,float _v_y,float _v_w);
void chassis_task();










#endif /* CHASSIS_H_ */
