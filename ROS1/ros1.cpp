/*
 * ros1.cpp
 *
 *  Created on: Aug 6, 2024
 *      Author: 88698
 */

#include "ros.h"
#include "../location/chassis_move.h" // not sure about the path
#include <ros1.h>
#include <chassis.h>

// extern float v_x, v_y, v_w, x, y, theta, cmd_v_x, cmd_v_y, cmd_v_w;
bool mission_complete = true;
bool receiveSpeedMode = false;
extern int coffee_table, cup_color;

ros::NodeHandle nh;

/** STM Publishers **/
//geometry_msgs::Twist chassis_current_speed;
//geometry_msgs::Pose chassis_current_pose;
std_msgs::Bool arriveDestination;
std_msgs::Bool receiveSpeedCmd;
//ros::Publisher pub_chassis("/odometry", &chassis_current_pose);
ros::Publisher pub_arriveDestination("/arrive_destination", &arriveDestination);
ros::Publisher pub_receiveSpeedCmd("/receive_speed_cmd", &receiveSpeedCmd);

/** STM Subscribers **/
ros::Subscriber<geometry_msgs::Twist> sub_chassis("/cmd_vel", ROS1::callback_Chassis);
ros::Subscriber<std_msgs::Bool> sub_missionFinish("/cmd_missionFinish", ROS1::callback_missonFinish);
ros::Subscriber<std_msgs::Int32> sub_coffeeTable("/cmd_coffeeTable", ROS1::callback_coffeeTable);
ros::Subscriber<std_msgs::Int32> sub_CupColor("/cmd_CupColor", ROS1::callback_CupColor);

//ros::Subscriber<std_msgs::Int32> sub_elevator("/cmd_elevator", ROS1::callback_Elevator);

//ros::Subscriber<std_msgs::Bool> sub_elevatorDoor("/cmd_elevatorDoor", ROS1::callback_ElevatorDoor);
//ros::Subscriber<std_msgs::Bool> sub_basketDoor("/cmd_basketDoor", ROS1::callback_BasketDoor);


namespace ROS1 {
  /**
   * @brief ROS1 節點宣告。
   * @param void
   */
  void init(void) {
    nh.initNode();

    //nh.advertise(pub_chassis);
    nh.advertise(pub_arriveDestination);
    nh.advertise(pub_receiveSpeedCmd);
    nh.subscribe(sub_chassis);
    nh.subscribe(sub_missionFinish);
    //nh.subscribe(sub_elevator);
    //nh.subscribe(sub_elevatorDoor);
    //nh.subscribe(sub_basketDoor);
    return;
  }

  /**
   * @brief ROS1 循環單位。
   * @param void
   */
  void spinCycle(void) {
    nh.spinOnce();
    return;
  }

  /**
   * @brief 底盤到達目的地時發送至 ROS 並呼叫任務機構。
   * @param void 
   */
  void pub_arrive_destination() {
    arriveDestination.data = achieve_flag;
    pub_arriveDestination.publish(&arriveDestination);
    return;
  }

  /**
   * @brief 底盤切換為接受ROS速度移動模式。
   * @param void
   */
  void pub_receive_speed_cmd() {
    receiveSpeedCmd.data = receiveSpeedMode;
    pub_receiveSpeedCmd.publish(&receiveSpeedCmd);
    return;
  }

  /**
   * @brief Chassis callback, 接收 ROS 底盤速度指令。
   * @param geometry_msgs::Twist
   */
  void callback_Chassis(const geometry_msgs::Twist &msg) {
    cmd_v_x = msg.linear.x;
    cmd_v_y = msg.linear.y;
    cmd_v_w = msg.angular.z;
    return;
  }

  /**
   * @brief MissionFinish callback, 接收任務完成資訊，完成後繼續底盤腳本。
   * @param std_msgs::Bool
   */
  void callback_missonFinish(const std_msgs::Bool &msg) {
    mission_complete = msg.data;
    return;
  }

  /**
   * @brief CoffeeTable callback, 接收咖啡桌號資訊。
   * @param std_msgs::Int32
   */
  void callback_coffeeTable(const std_msgs::Int32 &msg) {
    coffee_table = msg.data;
    return;
  }

  /**
   * @brief CupColor callback, 接收杯子顏色資訊。
   * @param std_msgs::Int32
   */
  void callback_CupColor(const std_msgs::Int32 &msg) {
    cup_color = msg.data;
    return;
  }

}
