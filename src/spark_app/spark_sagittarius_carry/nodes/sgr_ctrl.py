#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
Descripttion: Saggitarius moveit application action
version: 1.00
Author: shudong.hong@nxrobo.com
Company: NXROBO (深圳创想未来机器人有限公司)
LastEditors: shudong.hong@nxrobo.com
'''

import math
import numpy as np
import rospy
import sys
from moveit_msgs.msg import MoveGroupActionFeedback
import moveit_commander
import tf.transformations as transformations

import actionlib
from spark_sagittarius_carry.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult, SGRCtrlFeedback
from sdk_sagittarius_arm.srv import ServoRtInfo, ServoRtInfoRequest

ispython3 = True if sys.version_info.major == 3 else False


class MoveItSGRTool:
    def __init__(self, init_pose=True, end_effector=None):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_name = rospy.get_param("~robot_name", "sgr532")

        # 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
        self.cartesian = rospy.get_param('~cartesian', False)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm_group = moveit_commander.MoveGroupCommander('sagittarius_arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        self.gripper = moveit_commander.MoveGroupCommander(
            'sagittarius_gripper')

        # 当运动规划失败后，允许重新规划
        self.arm_group.allow_replanning(False)

        self.reference_frame = rospy.get_namespace()[1:] + 'base_link'
        # 设置目标位置所使用的参考坐标系
        self.arm_group.set_pose_reference_frame(self.reference_frame)

        # 设置目标位置所使用的参考坐标系
        self.gripper.set_pose_reference_frame(self.reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm_group.set_goal_position_tolerance(0.001)
        self.arm_group.set_goal_orientation_tolerance(0.001)

        self.gripper.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)

        # 设置末端效应点link的名称
        if (end_effector is not None):
            self.arm_group.set_end_effector_link(end_effector)

        # 获取终端link的名称
        self.end_effector_link = self.arm_group.get_end_effector_link()
        rospy.loginfo("end effector link: %s" % self.end_effector_link)

        #
        rospy.Subscriber("move_group/feedback", MoveGroupActionFeedback,
                         self._move_group_feedback_callback)
        self.moveit_group_status = "IDLE"

        # 控制机械臂先回到初始化位置
        if init_pose:
            self.arm_group.set_named_target('home')
            self.arm_group.go()
            rospy.sleep(1)
            self.pose_stay()

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def _move_group_feedback_callback(self, fb):
        self.moveit_group_status = fb.feedback.state

    def isPlanSuccess(self, px, py, pz, roll=0, pitch=0, yaw=0):
        self.arm_group.set_pose_target([px, py, pz, roll, pitch, yaw])
        plan = self.arm_group.plan()

        if ispython3:
            plan = plan[1]

        return len(plan.joint_trajectory.points) != 0

    def gripper_catch(self):
        self.gripper.set_joint_value_target([-0.021, -0.021])
        ret = self.gripper.go()
        if ret:
            rospy.sleep(1)
        return ret

    def gripper_open(self):
        self.gripper.set_joint_value_target([0.0, 0.0])
        ret = self.gripper.go()
        if ret:
            rospy.sleep(1)
        return ret

    def to_pose_eular(self, times, px, py, pz, roll=0, pitch=0, yaw=0):
        self.arm_group.set_pose_target([px, py, pz, roll, pitch, yaw])
        plan = self.arm_group.plan()

        if ispython3:  # python3 返回的是列表，下标1是 RobotTrajectory，也可以用列表中 error_code:MoveItErrorCodes 判断
            plan = plan[1]

        if len(plan.joint_trajectory.points) != 0:
            self.arm_group.execute(plan)
            rospy.sleep(times)
            return True
        else:
            rospy.logwarn("plan can not found!")
            return False

    def ee_target_offset(self, px, py, pz, roll=0, pitch=0, yaw=0, ee_type='grasp'):
        M = transformations.compose_matrix(
            angles=[roll, pitch, yaw], translate=[px, py, pz])
        if ee_type == 'grasp':
            M1 = np.dot(M, transformations.translation_matrix([-0.07, 0, 0]))
        else:
            M1 = M
        scale, shear, angles, translate, perspective = transformations.decompose_matrix(
            M1)
        return translate[0], translate[1], translate[2], angles[0], angles[1], angles[2]

    def ee_xyz_get_rpy(self, x, y, z):
        # Get yaw by Arctangent of x and y
        yaw = math.atan2(y, x)
        roll = 0
        pitch = 0

        # 使用三角函数求 pitch 的近似值
        # 模型：机械臂抽象成等腰三角形，机械臂末端到坐标原点的距离为底，机械臂臂长为两腰总和
        # 求底边角
        a = 0.532 / 2
        b = 0.532 / 2
        c = math.sqrt(x*x + y*y + z*z)
        if a + b <= c:
            # 三角形不成立
            pitch = 0
        else:
            # 已知三角边长求指定角度
            pitch = math.acos((a*a - b*b - c*c)/(-2*b*c)) - math.asin(z / c)

            # 限定范围
            if pitch > 1.57:
                pitch = 1.57
            elif pitch < 0:
                pitch = 0
        return roll, pitch, yaw

    def pose_stay(self):
        self.arm_group.set_joint_value_target(
            [0, 1.4, -1.48, 0, 1.6, 0])
        self.arm_group.go()

    def stop(self):
        self.arm_group.set_named_target('home')
        self.arm_group.go()
        rospy.sleep(1)
        self.arm_group.set_named_target('sleep')
        self.arm_group.go()
        rospy.sleep(1)
        self.gripper.set_joint_value_target([0, 0])
        self.gripper.go()
        rospy.sleep(0.5)


class CencelException(Exception):
    pass


class SGRCtrlActionServer:
    def __init__(self) -> None:
        self.sgr_tool = MoveItSGRTool()
        rospy.wait_for_service('get_servo_info', 3)
        self.servo_info_srv = rospy.ServiceProxy('get_servo_info', ServoRtInfo)
        self._server = actionlib.SimpleActionServer(
            'sgr_ctrl', SGRCtrlAction, self.execute, False)
        self._server.start()

    def execute(self, goal:SGRCtrlGoal):
        r = rospy.Rate(1)

        resp = SGRCtrlResult()
        fb = SGRCtrlFeedback()

        # 初始化夹爪
        fb.step = fb.EXEC_GRASP
        self._server.publish_feedback(fb)
        if goal.grasp_type is SGRCtrlGoal.GRASP_CLOSE:
            self.sgr_tool.gripper_catch()
        elif goal.grasp_type is SGRCtrlGoal.GRASP_OPEN:
            self.sgr_tool.gripper_open()

        # 开始规划
        fb.step = fb.PLANNING
        self._server.publish_feedback(fb)
        if goal.action_type is goal.ACTION_TYPE_DEFINE_STAY:
            # 指定动作
            fb.step = fb.EXEC_POSITION
            self._server.publish_feedback(fb)
            self.sgr_tool.pose_stay()
        else:
            if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_XYZ_RPY, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY, SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY]:
                # 指定末端姿态
                roll = goal.pos_roll
                pitch = goal.pos_pitch
                yaw = goal.pos_yaw
            else:
                # 动态计算末端姿态
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(goal.pos_x, goal.pos_y, goal.pos_z)
            x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(
                goal.pos_x, goal.pos_y, goal.pos_z,
                roll, pitch, yaw
            )

            # 检查姿态是否能到达
            try:
                if not self.sgr_tool.isPlanSuccess(x, y, z, roll, pitch, yaw):
                    raise
                if goal.action_type in [
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
                ]:
                    if not self.sgr_tool.isPlanSuccess(x, y, z + 0.04, roll, pitch, yaw):
                        raise
                    if not self.sgr_tool.isPlanSuccess(x, y, z + 0.12, roll, pitch, yaw):
                        raise
            except:
                rospy.logwarn("Plan could not found")
                resp.result = resp.PLAN_NOT_FOUND
                self._server.set_aborted(resp)
                return
            
            # 执行规划
            fb.step = fb.EXEC_POSITION
            self._server.publish_feedback(fb)
            try:
                # 抓取与放置动作
                if goal.action_type in [
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ,
                    SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
                ]:
                    if self._server.is_preempt_requested():
                        raise CencelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(1, x, y, z + 0.04, roll, pitch, yaw)

                    if self._server.is_preempt_requested():
                        raise CencelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z, roll, pitch, yaw)

                    fb.step = fb.EXEC_GRASP
                    self._server.publish_feedback(fb)
                    if self._server.is_preempt_requested():
                        raise CencelException("Preempt !!!")
                    if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PICK_XYZ, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY]:
                        self.sgr_tool.gripper_catch()
                    elif goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PUT_XYZ, SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY]:
                        self.sgr_tool.gripper_open()
                    
                    fb.step = fb.EXEC_POSITION
                    self._server.publish_feedback(fb)
                    if self._server.is_preempt_requested():
                        raise CencelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z + 0.12, roll, pitch, yaw)

                    # if self._server.is_preempt_requested():
                    #     raise CencelException("Preempt !!!")
                    # self.sgr_tool.pose_stay()

                    # 夹爪判断
                    if goal.action_type in [SGRCtrlGoal.ACTION_TYPE_PICK_XYZ, SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY]:
                        ret = self.servo_info_srv.call(ServoRtInfoRequest(servo_id=7))
                        if abs(ret.payload) < 24:
                            resp.result = resp.GRASP_FAILD
                            self._server.set_aborted(resp)
                            return

                # 执行某个动作
                else:
                    if self._server.is_preempt_requested():
                        raise CencelException("Preempt !!!")
                    self.sgr_tool.to_pose_eular(0.2, x, y, z, roll, pitch, yaw)
            except CencelException as e:
                rospy.logwarn(e)
                resp.result = resp.PREEMPT
                self._server.set_aborted()
            except Exception as e:
                rospy.logerr(e)
                resp.result = resp.ERROR
                self._server.set_aborted()
                
        resp.result = resp.SUCCESS
        self._server.set_succeeded(resp)


if __name__ == '__main__':
    rospy.init_node("sgr_ctrl_action", anonymous=True)
    rospy.sleep(5)

    ser = SGRCtrlActionServer()

    rospy.spin()
