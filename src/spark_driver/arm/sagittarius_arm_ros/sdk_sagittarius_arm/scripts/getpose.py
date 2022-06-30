#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy


class MoveItCartesianDemo:
    def __init__(self):
        #
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
        cartesian = rospy.get_param('~cartesian', False)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander('gripper')
        gripper1 = moveit_commander.MoveGroupCommander('gripper')
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('sagittarius_arm_link')

        # 设置目标位置所使用的参考坐标系
        gripper.set_pose_reference_frame('sagittarius_arm_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        gripper1.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        while not rospy.is_shutdown():
            # 获取当前位姿数据最为机械臂运动的起始位姿
            start_pose = arm.get_current_pose(end_effector_link).pose
            print(start_pose)
            rospy.sleep(0.1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
