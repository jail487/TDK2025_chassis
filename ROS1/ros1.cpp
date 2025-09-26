/*
 * ros1.cpp
 *
 *  Created on: Aug 6, 2024
 *      Author: 88698
 */

#include "ros.h"
#include <ros1.h>
#include <chassis.h>
#include "location.h"

extern float v_x, v_y, v_w, map_x, map_y, theta, cmd_v_x, cmd_v_y, cmd_v_w;
extern bool mission_flag, achieve_flag;
extern int coffeTable, cupColor;
extern int stage ;
extern float xDis_receive, yDis_receive;

// Rate limiting variables
static uint32_t last_pose_publish = 0;
static uint32_t last_destination_publish = 0;
static uint32_t last_speed_cmd_publish = 0;
static const uint32_t PUBLISH_INTERVAL_MS = 50; // 20Hz max publishing rate

ros::NodeHandle nh;

/** STM Publishers **/
//geometry_msgs::Twist chassis_current_speed;
geometry_msgs::Pose chassis_current_pose;
std_msgs::Int32 arriveDestination;
std_msgs::Int32 currentStage;
ros::Publisher pub_chassis("/odometry", &chassis_current_pose);
//ros::Publisher pub_arriveDestination("/arrive_destination", &arriveDestination);
ros::Publisher pub_currentStage("/current_stage", &currentStage);

/** STM Subscribers **/
ros::Subscriber<std_msgs::Int32> sub_chassis("/cmd_Xoffset", ROS1::callback_Xoffset);
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

    nh.advertise(pub_chassis);
   // nh.advertise(pub_arriveDestination);
    nh.advertise(pub_currentStage);
    
    nh.subscribe(sub_chassis);
    nh.subscribe(sub_missionFinish);
    nh.subscribe(sub_coffeeTable);
    nh.subscribe(sub_CupColor);
    return;
  }
int spin  = 0;
  /**
   * @brief ROS1 循環單位。
   * @param void
   */
  void spinCycle(void) {
    // Limit spinOnce frequency to 50Hz
      nh.spinOnce();
      spin++;

    return;
  }

  void pub_chassis_pose(void){
      chassis_current_pose.position.x = map_x;
      chassis_current_pose.position.y = map_y;
      chassis_current_pose.orientation.w = theta;
      pub_chassis.publish(&chassis_current_pose);
      return;
  }

  /**
   * @brief 底盤到達目的地時發送至 ROS 並呼叫任務機構。
   * @param void 
   */
//  void pub_arrive_destination() {
//      arriveDestination.data = achieve_flag;
//      pub_arriveDestination.publish(&arriveDestination);
//      return;
//  }

  /**
   * @brief 底盤切換為接受ROS速度移動模式。
   * @param void
   */
  void pub_current_stage() {
      if (ach()) {
        currentStage.data = stage;
      }
      else {
        currentStage.data = 0;
      }
      pub_currentStage.publish(&currentStage);
      return;
  }

  /**
   * @brief Chassis callback, 接收 ROS 底盤速度指令。
   * @param geometry_msgs::Twist
   */
  void callback_Xoffset(const std_msgs::Int32 &msg) {
      xDis_receive = msg.data;
      //cmd_v_w = msg.angular.z;
      return;
  }

  /**
   * @brief MissionFinish callback, 接收任務完成資訊，完成後繼續底盤腳本。
   * @param std_msgs::Bool
   */
  void callback_missonFinish(const std_msgs::Bool &msg) {
      mission_flag = msg.data;
      return;
  }

  /**
   * @brief CoffeeTable callback, 接收咖啡桌號資訊。
   * @param std_msgs::Int32
   */
  void callback_coffeeTable(const std_msgs::Int32 &msg) {
	  if(stage == 21){
      coffeTable = (int)msg.data;
	  }
      return;
  }

  /**
   * @brief CupColor callback, 接收杯子顏色資訊。
   * @param std_msgs::Int32
   */
  void callback_CupColor(const std_msgs::Int32 &msg) {
      cupColor = (int)msg.data;
      return;
  }

}
