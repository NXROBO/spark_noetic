/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2022, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <sdk_sagittarius_arm/ArmRadControl.h>
#include "sdk_sagittarius_arm/ArmInfo.h"

sensor_msgs::JointState joint_states;         

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}
#define MAX_SLAVER_ROBOT_NUM 5
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sagittarius_puppet_control");
  ros::NodeHandle n;
  int slaver_robot_num = 1;
  int i;
  ros::Publisher pub_positions[MAX_SLAVER_ROBOT_NUM];
  ros::Publisher pub_gripper[MAX_SLAVER_ROBOT_NUM];
  std::string robot_name_master, robot_name_slaver1,robot_name_slaver[MAX_SLAVER_ROBOT_NUM];
  ros::param::get("~robot_name_master", robot_name_master);
  ros::param::get("~slaver_robot_num", slaver_robot_num);
  if(slaver_robot_num > MAX_SLAVER_ROBOT_NUM)
  {
    ROS_ERROR("arm number must be not more than 5");
    return 1;
  }
  for(i=0; i<slaver_robot_num; i++)
  {
    ros::param::get("~robot_name_slaver"+std::to_string(i+1), robot_name_slaver[i]);
    pub_positions[i] = n.advertise<sdk_sagittarius_arm::ArmRadControl>(robot_name_slaver[i] + "/joint/commands", 100);
    pub_gripper[i] = n.advertise<std_msgs::Float64>(robot_name_slaver[i] + "/gripper/command", 100);

  }

  // 定阅主机械臂的关节角度
  ros::Subscriber sub_positions = n.subscribe(robot_name_master + "/joint_states", 100, joint_state_cb);
  ros::Publisher pub_torque = n.advertise<std_msgs::String>(robot_name_master + "/control_torque", 1);
  // 释放主机械臂的锁舵功能
  ros::ServiceClient srv_robot_info = n.serviceClient<sdk_sagittarius_arm::ArmInfo>(robot_name_master + "/get_robot_info");
  ros::Rate loop_rate(100);
  bool success;

  // 等待SDK节点开始订阅后，才开始发送。
  while ((pub_positions[slaver_robot_num-1].getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  while ((pub_torque.getNumSubscribers() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std_msgs::String msg;
  msg.data = "close";
  pub_torque.publish(msg);


  // 获取机械臂的关节数
  sdk_sagittarius_arm::ArmInfo robot_info_srv;
  success = srv_robot_info.call(robot_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get robot info.");
    return 1;
  }


  size_t cntr = 0;
  while (ros::ok())
  {
    sdk_sagittarius_arm::ArmRadControl pos_msg;
    for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
    {
      pos_msg.rad.push_back(joint_states.position.at(i));
    }
    for(i=0; i<slaver_robot_num; i++)
    {
      pub_positions[i].publish(pos_msg);
    }
    if (cntr == 5) 
    {
      std_msgs::Float64 gpr_msg;
      gpr_msg.data = joint_states.position.at(joint_states.position.size()-2);
      for(i=0; i<slaver_robot_num; i++)
      {
        pub_gripper[i].publish(gpr_msg);
      }
      cntr = 0;
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
