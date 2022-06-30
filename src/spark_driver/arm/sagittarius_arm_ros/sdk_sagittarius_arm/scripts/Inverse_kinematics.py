#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from copy import deepcopy
import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import math


def move2pose(times, px, py, pz, ox, oy, oz, ow):
    wpose = deepcopy(arm.get_current_pose(end_effector_link).pose)  # 拷贝对象
    wpose.position.x = px
    wpose.position.y = py
    wpose.position.z = pz
    wpose.orientation.x = ox
    wpose.orientation.y = oy
    wpose.orientation.z = oz
    wpose.orientation.w = ow
    arm.set_pose_target(wpose)  # 自由曲线
    arm.go()
    rospy.sleep(times)


def eular2orientation(pitch, yaw, roll):
    ox = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) + \
        math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    oy = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) + \
        math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    oz = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) - \
        math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    ow = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) - \
        math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    return ox, oy, oz, ow


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化ROS节点
    rospy.init_node('moveit_cartesian_demo', anonymous=True)

    # 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
    cartesian = rospy.get_param('~cartesian', False)

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('sagittarius_arm')

    # 初始化需要使用move group控制的机械臂中的gripper group
    gripper = MoveGroupCommander('sagittarius_gripper')
    gripper1 = moveit_commander.MoveGroupCommander('sagittarius_gripper')
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(False)

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

    # 归位
    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    # 获取当前位置为起始点
    pose = deepcopy(arm.get_current_pose(end_effector_link).pose)
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z
    ox = pose.orientation.x
    oy = pose.orientation.y
    oz = pose.orientation.z
    ow = pose.orientation.w
    while not rospy.is_shutdown(): # 运动到正方形四个点
        px -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        py -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        pz -= 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        py += 0.20
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

        pz += 0.1
        move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if rospy.is_shutdown():
            break

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
