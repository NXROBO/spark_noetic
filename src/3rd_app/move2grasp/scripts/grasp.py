#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import threading
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *

class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count, detect_color_blue
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        detect_color_blue=False
        self.is_found_object = False
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        thread1 = threading.Thread(target=self.image_cb,)
        thread1.setDaemon(True)
        thread1.start()
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        r1 = rospy.Rate(1)
        r1.sleep()
        pos = position()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)

    def grasp_cp(self, msg):
        global detect_color_blue
        rospy.loginfo("recvice grasp command:%s",msg.data)
        if msg.data == '1':
            # 订阅摄像头话题,对图像信息进行处理
            #self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
            detect_color_blue=True
            self.is_found_object = False
            rate = rospy.Rate(10)
            times=0
            steps=0
            while not self.is_found_object:
                rate.sleep()
                times+=1
                # 转一圈没有发现可抓取物体,退出抓取
                if steps>=5:
                    #self.sub.unregister()
                    print("stop grasp\n")
                    detect_color_blue=False
                    status=String()
                    status.data='-1'
                    self.grasp_status_pub.publish(status)
                    return
                # 旋转一定角度扫描是否有可供抓取的物体
                if times>=30:
                    times=0
                    steps+=1
                    self.turn_body()
                    print("not found\n")
            print("unregisting sub\n")
            # 抓取检测到的物体 
            detect_color_blue=False   
            self.grasp()
            status=String()
            status.data='1'
            self.grasp_status_pub.publish(status) 
        if msg.data=='0':
            # 放下物体
            self.is_found_object = False
            detect_color_blue=False
            self.release_object()
            status=String()
            status.data='0'
            
            self.grasp_status_pub.publish(status)

    # 执行抓取
    def grasp(self):
        rospy.loginfo("start to grasp\n")
        global xc, yc, found_count
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        print(s)
        arr=s.split()
        a1=arr[0]
        a2=arr[1]
        a3=arr[2]
        a4=arr[3]
        a = [0]*2
        b = [0]*2
        a[0]=float(a1)
        a[1]=float(a2)
        b[0]=float(a3)
        b[1]=float(a4)
        print('k and b value:',a[0],a[1],b[0],b[1])
        r1 = rospy.Rate(0.095)
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = a[0] * yc + a[1]
        pos.y = b[0] * xc + b[1]
        pos.z = 20
         # pos.z = 20
        print("z = 20\n")
        self.pub1.publish(pos)
        r2.sleep()
        # go down -100
        pos.z = -50
        self.pub1.publish(pos)
        print("z = -83\n")
        r2.sleep()

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()

        # 提起物体
        pos.x = 200  #160
        pos.y = 0
        pos.z = 155  #55
        self.pub1.publish(pos)
        r1.sleep()

    def hsv_value(self):
        try:
            filename = os.environ['HOME'] + "/color_block_HSV.txt"
            with open(filename, "r") as f:
                for line in f:
                    split = line.split(':')[1].split(' ')
                    lower = split[0].split(',')
                    upper = split[1].split(',')
                    for i in range(3):
                        lower[i] = int(lower[i])
                        upper[i] = int(upper[i])

            lower = np.array(lower)
            upper = np.array(upper)
        except:
            raise IOError('could not find hsv_value file : {},please execute #13 command automatically '
                          'generate this file'.format(filename))

        return lower, upper

    # 使用CV检测物体       
    def image_cb(self):
        global xc, yc, xc_prev, yc_prev, found_count, detect_color_blue	
        capture = cv2.VideoCapture(0)
        LowerBlue, UpperBlue = self.hsv_value()

        while True:
            # time.sleep(0.08)
            ret,frame = capture.read()
            cv_image2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)		
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (9,9), 0)
            # detect contour
            cv2.imshow("win2", mask)
            cv2.waitKey(1)
            if detect_color_blue :
                _, contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    size = []
                    size_max = 0
                    distance_list = []
                    max_distance = 450
                    for i, c in enumerate(contours):
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        x_mid, y_mid = rect[0]

                        w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
                        h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)

                        size.append(w * h)
                        # 所有点到spark的距离
                        distance_list.append(math.sqrt((320 - x_mid) ** 2 + (300 - y_mid) ** 2))

                        if size[i] > size_max and distance_list[i] < max_distance:
                            size_max = size[i]
                            min_distance = distance_list[i]
                            index = i
                            xc = x_mid
                            yc = y_mid
                            cv2.circle(frame, (np.int32(xc), np.int32(yc)), 2, (255, 0, 0), 2, 8, 0)
                    if found_count >= 15 and min_distance < 350:
                        self.is_found_object = True
                        cmd_vel = Twist()
                        self.cmd_vel_pub.publish(cmd_vel)
                    else:
                        # if box is not moving
                        if abs(xc - xc_prev) <= 50 and abs(yc - yc_prev) <= 50 and yc > 150 and yc < 370 and xc > 100 and xc < 540:
                            found_count = found_count + 1
                        else:
                            found_count = 0

                else:
                    found_count = 0
            xc_prev = xc
            yc_prev = yc
            cv2.imshow("win1", frame)
            cv2.waitKey(1)
            #key = cv2.waitKey(10)
 
    # 释放物体
    def release_object(self):
        r1 = rospy.Rate(1)
        r2 = rospy.Rate(1)
        pos = position()
        # go forward
        pos.x = 200
        pos.y = 0
        pos.z = -40
        self.pub1.publish(pos)
        r1.sleep()

        # stop pump
        self.pub2.publish(0)
        r2.sleep()
        r1.sleep()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)
        r1.sleep()
        return 'succeeded'
    # 转动机器人到一定角度       
    def turn_body(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.25
        rate = rospy.Rate(10)
        for i in range(40):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
       

        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

