#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


import os
import time 
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import threading
import thread
import string
import math
import numpy as np
from std_msgs.msg import String
from spark_fetchtea.srv import *
# from cv_bridge import CvBridge, CvBridgeError

class VoiceMaster():
    '''
    监听语音的主控，用于演示触发端茶，回家的功能
    '''
    def __init__(self):
        '''
        初始化
        '''        
        self.command_sub = rospy.Subscriber("/voice/stt", String, self.voicecommand_cb, queue_size=1)  
        self.fetchtea_client = rospy.ServiceProxy('s_fetchtea', scene)
        self.gohome_client = rospy.ServiceProxy('s_gohome', scene)
        
    def voicecommand_cb(self, asr_msg):
        command_str = asr_msg.data;
        print 'command:'
        print command_str;
        if command_str == '去端茶':
            rospy.loginfo('CMD send goal')
            self.call_fetchtea_service(1,'goal')
        elif command_str == '回家':
            rospy.loginfo('CMD send home')
            self.call_fetchtea_service(1,'home')
        else:
            print "invalid command"
        
    def call_fetchtea_service(self, type,param):
        '''
        自主行为服务控制
        :param type:1,表示启动;0,表示关闭
        '''
        rospy.wait_for_service('s_fetchtea')
        try:            
            p = param 
            respon = self.fetchtea_client(type, param);
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False
        
if __name__ == '__main__':
    try:
        rospy.init_node('spark_fetch_tea_main', anonymous=False)
        rospy.loginfo("Init spark fetch tea main")   
        VoiceMaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark fetch tea main")

