#!/usr/bin/env python3
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
