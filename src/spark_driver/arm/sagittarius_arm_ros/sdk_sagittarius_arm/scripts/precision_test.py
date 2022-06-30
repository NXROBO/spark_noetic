#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    global arm
    global cartesian
    global end_effector_link
    
    def move2pose(self, times, px, py, pz, ox, oy, oz, ow):
        global arm
        global cartesian
        global end_effector_link
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        print(start_pose)        
        # 初始化路点列表
        waypoints = []
 
		# 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints.append(start_pose)
            
            # 设置路点数据，并加入路点列表，所有的点都加入
        wpose = deepcopy(start_pose)#拷贝对象

        wpose.position.z -= 0.25
        print("到绿方块的上方")
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x =  ox
        wpose.orientation.y =  oy
        wpose.orientation.z =  oz
        wpose.orientation.w =  ow
        print(wpose)
        if cartesian:  #如果设置为True，那么走直线
            waypoints.append(deepcopy(wpose))
        else:          #否则就走曲线
            arm.set_pose_target(wpose)  #自由曲线
            arm.go()
            rospy.sleep(times*0.95)

    def __init__(self):
        global arm
        global cartesian
        global end_effector_link
        #
        moveit_commander.roscpp_initialize(sys.argv)

        #初始化ROS节点  
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
        arm.set_pose_reference_frame('world')

        # 设置目标位置所使用的参考坐标系
        gripper.set_pose_reference_frame('world')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        gripper1.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        # 控制机夹爪打开
        ''' gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(1)

        # 控制机夹爪关闭
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(1)
        print("gripper open") 
        # 控制机夹爪打开
        gripper1.set_named_target('open')
        gripper1.go()
        rospy.sleep(3) '''
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        # 夹起来
        print("夹起来")
        gripper1.set_joint_value_target([-0.005, -0.005])
        gripper1.go()
        rospy.sleep(1)        #print("gripper set_joint_value_target([0.1,0.1])")
        # 设置夹爪的目标位置，并控制夹爪运动
        #gripper1.set_joint_value_target([-0.01, -0.01])
        #gripper1.go()
        #rospy.sleep(1)
# position: 
#   x: 0.1
#   y: 0.15
#   z: 0.1
# orientation: 
#   x: 0
#   y: 0.689296120492
#   z: 0.00956006494475
#   w: 0.723968564058
        while not rospy.is_shutdown():
            if not rospy.is_shutdown():
                self.move2pose(1, 0.15, -0.1, 0.25, 0, 0, 0, 1)
            else:
                break
            if not rospy.is_shutdown():
                self.move2pose(1, 0.25, -0.1, 0.25, 0, 0, 0, 1)
            else:
                break
            

        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        arm.set_named_target('sleep')
        arm.go()
        rospy.sleep(1)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass

