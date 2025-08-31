/*
 * ros1.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 88698
 */

#ifndef ROS_1_H_
#define ROS_1_H_

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#define ROS_PUB_FREQUENCY 10

namespace ROS1 {
    void init(void);
    void spinCycle(void);

    // STM Publishers
    void pub_arrive_destination();
    void pub_receive_speed_cmd();
    void pub_chassis_pose();

    // STM Subscribers
    void callback_Chassis(const geometry_msgs::Twist &msg);
    void callback_missonFinish(const std_msgs::Bool &msg);
    void callback_coffeeTable(const std_msgs::Int32 &msg);
    void callback_CupColor(const std_msgs::Int32 &msg);
}

#endif
