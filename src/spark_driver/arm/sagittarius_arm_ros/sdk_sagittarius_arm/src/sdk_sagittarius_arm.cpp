/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sagittarius_arm_node");
    ROS_INFO("sagittarius_arm bring up");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string robot_name, robot_model;
    ros::param::get("~robot_name", robot_name);
    ros::param::get("~robot_model", robot_model);
    sdk_sagittarius_arm::SagittariusArmReal sar(nh, pnh, robot_name, robot_model);
    ROS_INFO("Sagittarius driver is running");
    ros::spin();

    return 0;
}






