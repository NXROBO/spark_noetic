#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# BSD 3-Clause License

# Copyright (c) 2025, NXROBO

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import math
 
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    #初始化ROS节点  
    rospy.init_node('moveit_cartesian_demo', anonymous=True)

    # 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
    cartesian = rospy.get_param('~cartesian', False)
                    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('sagittarius_arm')

    # 初始化需要使用move group控制的机械臂中的gripper group
    gripper = MoveGroupCommander('sagittarius_gripper')
    gripper1 = moveit_commander.MoveGroupCommander('sagittarius_gripper')
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('world')

    # 设置目标位置所使用的参考坐标系
    gripper.set_pose_reference_frame('world')
            
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.0001)

    gripper1.set_goal_joint_tolerance(0.001)
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    while not rospy.is_shutdown():
        print("\nset joint 2: 0° -> 45°")
        print("set joint 3: 0° -> -45°")
        tjoint = [0, math.pi / 4, -math.pi / 4, 0, 0, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        print("\nset joint 4: 0° -> 90°")
        print("set joint 6: 0° -> 90°")
        tjoint = [0, math.pi / 4, -math.pi / 4, math.pi / 2, 0, math.pi / 2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        print("\nset joint 4: 90° -> -90°")
        print("set joint 6: 90° -> -90°")
        tjoint = [0, math.pi / 4, -math.pi / 4, -math.pi / 2, 0, -math.pi / 2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        print("\nset joint 3: 0° -> 90°")
        print("set joint 5: 0° -> -90°")
        tjoint = [0, 0, math.pi / 2, 0, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        print("\nset joint 4: 0° -> 45°")
        tjoint = [0, 0, math.pi / 2, math.pi / 4, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        print("\nset joint 4: 45° -> -45°")
        tjoint = [0, 0, math.pi / 2, -math.pi / 4, -math.pi / 2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        rospy.sleep(0.5)
        if rospy.is_shutdown():
            break
        print(arm.get_current_pose(end_effector_link).pose)

        break

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)
    arm.set_named_target('sleep')
    arm.go()
    rospy.sleep(1)
    gripper1.set_joint_value_target([0, 0])
    gripper1.go()
    rospy.sleep(0.5)

            
    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

