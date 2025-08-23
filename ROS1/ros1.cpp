/*
 * ros1.cpp
 *
 *  Created on: Aug 6, 2024
 *      Author: pomelo925
 */

#include "ros.h"
#include <ros1.h>
#include <chassis.h>

extern float map_x,map_y,theta,cmd_v_x,cmd_v_y,cmd_v_w;
ros::NodeHandle nh;

/** STM Publishers **/
geometry_msgs::Twist chassis_current_speed;
geometry_msgs::Pose chassis_current_pose;
ros::Publisher pub_chassis("/odometry", &chassis_current_pose);

/** STM Subscribers **/
ros::Subscriber<geometry_msgs::Twist> sub_chassis("/cmd_vel", ROS1::callback_Chassis);
ros::Subscriber<std_msgs::Bool> sub_intake("/cmd_intake", ROS1::callback_Intake);
ros::Subscriber<std_msgs::Int32> sub_elevator("/cmd_elevator", ROS1::callback_Elevator);

ros::Subscriber<std_msgs::Bool> sub_elevatorDoor("/cmd_elevatorDoor", ROS1::callback_ElevatorDoor);
ros::Subscriber<std_msgs::Bool> sub_basketDoor("/cmd_basketDoor", ROS1::callback_BasketDoor);


namespace ROS1 {
  /**
   * @brief ROS1 節點宣告。
   * @param void
   */
  void init(void){
    nh.initNode();

    nh.advertise(pub_chassis);

    nh.subscribe(sub_chassis);
   // nh.subscribe(sub_intake);
    //nh.subscribe(sub_elevator);
    //nh.subscribe(sub_elevatorDoor);
    //nh.subscribe(sub_basketDoor);
    return;
  }


  /**
   * @brief ROS1 循環單位。
   * @param void
   */
  void spinCycle(void){
    nh.spinOnce();
    return;
  }


  /**
   * @brief STM 發佈底盤速度至 ROS。
   * @param void
   */
  void pub_chassis_pose(void){
    chassis_current_pose.position.x = map_x;
    chassis_current_pose.position.y = map_y;
    chassis_current_pose.orientation.w = theta;
    pub_chassis.publish(&chassis_current_pose);
    return;
  }



  /**
   * @brief Chassis 回調函數。
   * @param geometry_msgs::Twist
   */
  void callback_Chassis(const geometry_msgs::Twist &msg){
    cmd_v_x = msg.linear.x;
    cmd_v_y = msg.linear.y;
    cmd_v_w = msg.angular.z;
    return;
  }


  /**
   * @brief Intake 回調函數。
   * @param std_msgs::Bool
   */
  void callback_Intake(const std_msgs::Bool &msg){
    // if(msg.data) runIntake = true;
    // else runIntake = false;
    return;
  }



  /**
   * @brief Elevator 回調函數。
   * @param std_msgs::Int32
   */
  void callback_Elevator(const std_msgs::Int32 &msg){
    // runElevator = msg.data;
    return;
  }



  /**
   * @brief ElevatorDoor 回調函數。
   * @param std_msgs::Bool
   */
  void callback_ElevatorDoor(const std_msgs::Bool &msg){
    // if(msg.data) runElevatorDoor = true;
    // else runElevatorDoor = false;
    return;
  }


  /**
   * @brief BasketDoor 回調函數。
   * @param std_msgs::Bool
   */
  void callback_BasketDoor(const std_msgs::Bool &msg){
    // if(msg.data) runBasketDoor = true;
    // else runBasketDoor = false;
    return;
  }
}
