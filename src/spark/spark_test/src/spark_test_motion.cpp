/* BSD 3-Clause License

Copyright (c) 2016 ~ 2025, NXROBO Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "ros/ros.h"
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MotionTest
{
public:
  ros::NodeHandle nhandle;
  boost::thread* test_motion_thread_;
  ros::Publisher cmd_pub;
  MotionTest(ros::NodeHandle in_nh)
  {
    nhandle = in_nh;

    cmd_pub = nhandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    test_motion_thread_ = new boost::thread(boost::bind(&MotionTest::testMotion, this));
  }

  ~MotionTest()
  {
    if (test_motion_thread_ != NULL)
    {
      delete test_motion_thread_;
    }
  }

  bool testMotion()
  {
    ros::Rate rate(10);
    double prev_sec = ros::Time().now().toSec();
    int sec = 10;
    while (1)
    {
      //逆时针旋转
      if (ros::Time().now().toSec() - prev_sec > sec)
        break;
      ROS_INFO("ni shi zhen");
      testTurnBody(0, 1, 10);
      rate.sleep();
    }
    prev_sec = ros::Time().now().toSec();
    while (1)
    {
      //顺时针旋转
      if (ros::Time().now().toSec() - prev_sec > sec)
        break;
      ROS_INFO("shen shi zhen");
      testTurnBody(0, -1, 10);
      rate.sleep();
    }
    prev_sec = ros::Time().now().toSec();
    while (1)
    {
      //逆时针同心圆旋转
      if (ros::Time().now().toSec() - prev_sec > sec)
        break;
      ROS_INFO("nishizhen tongxinyuan");
      testTurnBody(0.2, 1, 10);
      rate.sleep();
    }

    prev_sec = ros::Time().now().toSec();
    while (1)
    {
      //顺时针同心圆旋转
      if (ros::Time().now().toSec() - prev_sec > sec)
        break;
      ROS_INFO("shunshizhen tongxinyuan");
      testTurnBody(0.2, -1, 10);
      rate.sleep();
    }
  }

  void testTurnBody(float linearx, float angularz, float sec)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = linearx;
    vel.angular.z = angularz;
    cmd_pub.publish(vel);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spark_test_node");
  ROS_INFO("Spark test bring up");

  ros::NodeHandle n;
  MotionTest test(n);

  // start to test the spark
  // test.testTopicStatus();
  ROS_INFO("Spark test stop");
  ros::spin();

  return 0;
}
