#!/usr/bin/env python3
import sys
import rospy
import time
import _thread
from std_msgs.msg import String

import actionlib
from spark_sagittarius_carry.msg import SGRCtrlAction, SGRCtrlGoal
start_cali = 0
go_cali_pos = 0
next_cali_pos = 0


def msg_callback(data):
    global start_cali
    global go_cali_pos
    global next_cali_pos

    if(data.data == "start"):
        start_cali = 1
    elif(data.data == "go"):
        go_cali_pos = 1
    elif(data.data == "next"):
        next_cali_pos = 1


def talker(threadName, delay):
    global start_cali
    global go_cali_pos
    global next_cali_pos

    # pub1 = rospy.Publisher('position_write_topic', position, queue_size=5)
    client = actionlib.SimpleActionClient(
        rospy.get_param("~arm_name", "sgr532") + '/' + 'sgr_ctrl', SGRCtrlAction)
    client.wait_for_server()
    pub2 = rospy.Publisher('cali_pix_topic', String, queue_size=5)
    sub3 = rospy.Subscriber("cali_arm_cmd_topic", String, msg_callback)
    goal = SGRCtrlGoal()
    goal_stay = SGRCtrlGoal()
    r1 = rospy.Rate(1)  # 1s
    r2 = rospy.Rate(0.05)  # 20s
    r3 = rospy.Rate(0.2)  # 5s
    r4 = rospy.Rate(0.4)  # 2.5s
    r1.sleep()

    goal_stay.grasp_type = goal_stay.GRASP_OPEN
    goal_stay.action_type = goal_stay.ACTION_TYPE_DEFINE_STAY
    client.send_goal_and_wait(goal_stay, rospy.Duration.from_sec(30))

    goal.action_type = goal.ACTION_TYPE_XYZ_RPY
    goal.pos_x = 0.28
    goal.pos_z = -0.12
    goal.pos_pitch = 1.57
    
    if rospy.get_param("~fast_mode", True):
        # point_list = [
        #     [0.28, 0],
        #     [0.22, -0.1],
        #     [0.35, -0.1],
        #     [0.35, 0],
        #     [0.35, 0.1],
        #     [0.22, 0.1]
        # ]
        point_list = [
            [0.28, 0],
            [0.22, 0],
            [0.35, -0.1],
            [0.35, 0.1]
        ]
    else:
        point_list = [
            [0.28, 0], # 第一次用于引导用户调整摄像头位置的姿态
            [0.28, 0],
            [0.22, 0],
            [0.22, -0.1],
            [0.28, -0.1],
            [0.35, -0.1],
            [0.35, 0],
            [0.35, 0.1],
            [0.28, 0.1],
            [0.22, 0.1]
        ]

    print("Calibration pose node wait to start----")
    while(start_cali == 0):
        r1.sleep()
    if(start_cali == 0):
        return

    client.send_goal_and_wait(goal, rospy.Duration.from_sec(30))
    while(go_cali_pos == 0):
        r1.sleep()
    if(go_cali_pos == 0):
        return
    go_cali_pos = 0
    client.send_goal_and_wait(goal_stay, rospy.Duration.from_sec(30))
    pub2.publish("")
    while(next_cali_pos == 0):
        r1.sleep()
    if(next_cali_pos == 0):
        return
    next_cali_pos = 0

    print("start to move the sagittarius----")
    for i in range(1, len(point_list)):
        if rospy.is_shutdown():
            break
        # r1.sleep()

        # 移动到标定位置
        goal.action_type = goal.ACTION_TYPE_PICK_XYZ_RPY
        goal.pos_x = point_list[i - 1][0]
        goal.pos_y = point_list[i - 1][1]
        client.send_goal_and_wait(goal, rospy.Duration.from_sec(30))

        goal.action_type = goal.ACTION_TYPE_PUT_XYZ_RPY
        goal.pos_x = point_list[i][0]
        goal.pos_y = point_list[i][1]
        client.send_goal_and_wait(goal, rospy.Duration.from_sec(30))
        client.send_goal_and_wait(goal_stay, rospy.Duration.from_sec(30))
        print("Point %d: %.4f, %.4f" % (i + 1, goal.pos_x, goal.pos_y))
        pub2.publish("")

        # 等待标定数据采集后进行下一个动作
        while(next_cali_pos == 0):
            r1.sleep()
        if(next_cali_pos == 0):
            return
        next_cali_pos = 0

        r4.sleep()

    print("end move----")
    r3.sleep()


if __name__ == '__main__':
    rospy.init_node('cali_pos', anonymous=True)
    # sub3 = rospy.Subscriber("cali_arm_cmd_topic", String, msg_callback)
    _thread.start_new_thread(talker, ("Thread-1", 2, ))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
