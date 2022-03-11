#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
import logging
import time
from threading import Thread, Event
from lib import Microphone,AipSpeech

""" 你的 APPID AK SK """
APP_ID = '23797700'
API_KEY = 'dK8rz5DG9pPi90cgQVtx1ddn'
SECRET_KEY = 'X1lzayj5RQwfkoLHXHd5DALRvFZpjhBP'

def main():
    #logging.basicConfig(level=logging.DEBUG)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
    rospy.init_node('ali_asr', anonymous=True)
    while not rospy.is_shutdown():
        data = mic.listen()
        result = client.asr(b''.join(list(data)), 'pcm', 16000, {'dev_pid': 1537,})
        #print (result)
        if result['err_no']==0 and len(result["result"][0])>1:
            text = result["result"][0]
            pub.publish(text)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
