#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import os
import yaml
#import pandas as pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression
#from PIL import Image, ImageDraw, ImageFont
xc = 0
yc = 0

count = 9
index = 0
# first_findContours = True
# contours = []

arm_cmd_sub = None

xarray = None
yarray = None
xarray_list = None
yarray_list = None
xc_array = None
yc_array = None

cali_origin_x = 335
cali_origin_y = 320
cali_w = 15
cali_h = 15
collect_times = 200
start_flag = 1
put_cube = 1
move_position = 0
collecting = 0
c_cnt = 0
HSV_value = [0, 0, 0]
lower_HSV = np.array([0, 0, 0])
upper_HSV = np.array([0, 0, 0])


def mean_hsv(img, hsv_value):
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_value[0] += np.mean(HSV[:, :, 0])
    hsv_value[1] += np.mean(HSV[:, :, 1])
    hsv_value[2] += np.mean(HSV[:, :, 2])
    return hsv_value


def hsv_range(hsv_value):

    H_range = 6
    S_range = 120
    V_range = 120

    lower_H = int(hsv_value[0] - H_range)
    upper_H = int(hsv_value[0] + H_range)

    lower_S = int(hsv_value[1] - S_range)
    upper_S = int(hsv_value[1] + S_range)

    lower_V = int(hsv_value[2] - V_range)
    upper_V = int(hsv_value[2] + V_range)

    if lower_H < 0:
        lower_H = 0
    if upper_H > 180:
        upper_H = 180

    if lower_S < 50:
        lower_S = 50
    if upper_S > 255:
        upper_S = 255

    if lower_V < 50:
        lower_V = 50
    if upper_V > 255:
        upper_V = 255

    lower_HSV = np.array([lower_H, lower_S, lower_V])
    upper_HSV = np.array([upper_H, upper_S, upper_V])
    return lower_HSV, upper_HSV


def test(lower_HSV, upper_HSV, image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_HSV, upper_HSV)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)
    cv2.putText(mask, 'Done! Press q to exit!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("HSV_img", mask)


def image_callback(data):
    global xc, yc
    global move_position
    global collecting
    global c_cnt
    global HSV_value
    global lower_HSV, upper_HSV
    global first_findContours
    global contours

    if start_flag:
        return
    # change to opencv
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv_image_draw = cv_image1.copy()
    if(put_cube):
        cv2.putText(cv_image_draw, 'please place the blue cube directly under', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'the gripper of the robotic arm!', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'then press the \'ENTER\' ', (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'in another terminal!', (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        c_cnt = 0
        cv2.imshow("win_draw", cv_image_draw)
        cv2.waitKey(1)
        return
    elif(move_position):
        cv2.putText(cv_image_draw, 'please move the camera to make ', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'the blue cube color in the rectangle ', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'green box! then press the \'ENTER\' ', (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, 'in another terminal!', (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(cv_image_draw, (cali_origin_x, cali_origin_y),
                      (cali_origin_x + cali_w, cali_origin_y + cali_h), (0, 255, 0), 3)
        c_cnt = 0
        cv2.imshow("win_draw", cv_image_draw)
        cv2.waitKey(1)
        return
    elif(collecting):
        c_cnt = c_cnt+1
        cv2.putText(cv_image_draw, 'collecting the hsv!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image_draw, ' please wait for 5s.', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(cv_image_draw, (cali_origin_x, cali_origin_y),
                      (cali_origin_x + cali_w, cali_origin_y + cali_h), (0, 255, 0), 3)
        frame = cv_image1[cali_origin_y + 5:cali_origin_y +
                          cali_h-5, cali_origin_x + 5:cali_origin_x + cali_w-5]
        HSV_value = mean_hsv(frame, HSV_value)
        cv2.imshow("win_draw", cv_image_draw)
        cv2.waitKey(1)

        if(c_cnt >= collect_times):
            for i in range(len(HSV_value)):
                HSV_value[i] = HSV_value[i] / collect_times
                print(i, len(HSV_value), HSV_value[i])
            lower_HSV, upper_HSV = hsv_range(HSV_value)
            #save_hsv(name, lower_HSV, upper_HSV)
            print(lower_HSV, upper_HSV)
            collecting = 0
            cv2.destroyWindow("win_draw")
            rospy.Subscriber("cali_pix_topic", String, command_callback)
            arm_cmd_sub.publish(String("next"))
        return
    # test(lower_HSV, upper_HSV, cv_image1)

    cv_image_cp = cv_image1.copy()
    cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
    cv_image_gray = cv2.inRange(cv_image_hsv, lower_HSV, upper_HSV)
    # smooth and clean noise
    cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
    cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
    # detect contour
    cv2.imshow("win1", cv_image1)
    cv2.imshow("win2", cv_image_gray)
    cv2.waitKey(1)
    contours, hier = cv2.findContours(
        cv_image_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    size = []
    size_max = 0
    for i, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
        y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
        w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
        h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
        size.append(w * h)
        if size[i] > size_max:
            size_max = size[i]
            index = i
            xc = x_mid
            yc = y_mid

    # if first_findContours:
    #     # for i in list(enumerate(contours))[index][1]:
    #     #     contours.append("%d,%d" % (i[0][0], i[0][1]))
    #     contours = ["%d,%d" % (int(i[0][0]), int(i[0][1])) for i in list(enumerate(contours))[index][1]]
    #     rospy.loginfo(contours)
    #     first_findContours = False
    # cv2.imshow("win1", cv_image1)


def command_callback(data):
    global xc_array
    global yc_array
    global xarray
    global yarray
    global index
    global lower_HSV, upper_HSV
    # s = '' + str(xc) + ' ' + str(yc) + '\n'
    # file_pix = open('thefile.txt', 'a')
    # file_pix.write(s)
    # file_pix.close()
    # print(xc, yc)
    if index < count:
        xc_array[index] = xc
        yc_array[index] = yc
        xarray[index] = xarray_list[index]
        yarray[index] = yarray_list[index]
        print("%d/%d,pose x,y: %.4f,%.4f. cam x,y: %d,%d" %
              (index+1, count, xarray[index], yarray[index], xc, yc))
        # reshape to 2D array for linear regression
        xc_array = xc_array.reshape(-1, 1)
        yc_array = yc_array.reshape(-1, 1)
        xarray = xarray.reshape(-1, 1)
        yarray = yarray.reshape(-1, 1)
        index = index + 1
        arm_cmd_sub.publish(String("next"))
    if index == count:
        Reg_x_yc = LinearRegression().fit(yc_array, xarray)
        Reg_y_xc = LinearRegression().fit(xc_array, yarray)
        k1 = Reg_x_yc.coef_[0][0]
        b1 = Reg_x_yc.intercept_[0]
        k2 = Reg_y_xc.coef_[0][0]
        b2 = Reg_y_xc.intercept_[0]
        s = '' + str(k1) + ' ' + str(b1) + ' ' + str(k2) + ' ' + str(b2) + '\n'
        
        # contours.append('\n')
        # s2 = ' '.join(contours)
        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'w')
        file_pix.write(s)
        # file_pix.write(s2)
        file_pix.close()
        print("calibration params path: " + filename)
        print("Linear Regression for x and yc is :  x = %.5fyc + (%.5f)" % (k1, b1))
        print("Linear Regression for y and xc is :  y = %.5fxc + (%.5f)" % (k2, b2))
        index = 0
        try:
            with open(rospy.get_param("~hsv_config", ""), "w") as f:
                hsv_dist = {
                    'h_max':upper_HSV[0].item(),
                    'h_min':lower_HSV[0].item(),
                    's_max':upper_HSV[1].item(),
                    's_min':lower_HSV[1].item(),
                    'v_max':upper_HSV[2].item(),
                    'v_min':lower_HSV[2].item()
                }
                yaml.dump(hsv_dist, f)
        except:
            rospy.logwarn("can't open hsv config file")
            rospy.logwarn("HSV  (upper, lower)")
            rospy.logwarn("H    (%3d, %3d)" % (upper_HSV[0].item(), lower_HSV[0].item()))
            rospy.logwarn("S    (%3d, %3d)" % (upper_HSV[1].item(), lower_HSV[1].item()))
            rospy.logwarn("V    (%3d, %3d)" % (upper_HSV[2].item(), lower_HSV[2].item()))
        print("******************************************************")
        print("     finish the calibration. Press ctrl-c to exit     ")
        print("             标定完成. 然后 Ctrl-C 退出程序             ")
        print("******************************************************")


def msg_callback(data):
    global start_flag
    global put_cube
    global move_position
    global collecting
    if(data.data == "start"):
        start_flag = 0
        put_cube = 1
    elif(data.data == "start2"):
        put_cube = 0
        move_position = 1
    elif(data.data == "start3"):
        move_position = 0
        collecting = 1


def main():
    global arm_cmd_sub
    global xarray_list
    global yarray_list
    global xarray
    global yarray
    global xc_array
    global yc_array
    global count
    rospy.init_node('image_converter', anonymous=True)
    r1 = rospy.Rate(1)  # 1s

    if rospy.get_param("~fast_mode", True):
        # xarray_list = [0.22, 0.35, 0.35, 0.35, 0.22]
        # yarray_list = [-0.1, -0.1, 0, 0.1, 0.1]
        # count = 5
        xarray_list = [0.22, 0.35, 0.35]
        yarray_list = [0, -0.1, 0.1]
        count = 3
    else:
        xarray_list = [0.28, 0.22, 0.22, 0.28, 0.35, 0.35, 0.35, 0.28, 0.22]
        yarray_list = [0, 0, -0.1, -0.1, -0.1, 0, 0.1, 0.1, 0.1]
        count = 9
    xarray = np.zeros(count)
    yarray = np.zeros(count)
    xc_array = np.zeros(count)
    yc_array = np.zeros(count)

    print("Calibration node wait to start----")
    # while(start_flag == 0):
    #     r1.sleep()
    # if(start_flag == 0):
    #     return

    sub1 = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    # sub2 = rospy.Subscriber("cali_pix_topic", String, command_callback)
    sub3 = rospy.Subscriber("cali_cmd_topic", String, msg_callback)
    arm_cmd_sub = rospy.Publisher('cali_arm_cmd_topic', String, queue_size=5)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
