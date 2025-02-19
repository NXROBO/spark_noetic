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
from lib import Microphone

import urllib3
import urllib
import json
import hashlib
import base64
import sys
import string
if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

def stt(data):
    tmp=[]
    for d in data:
        tmp.append(d)
    base64_audio = base64.b64encode(''.join(tmp))
    body = urllib.urlencode({'audio': base64_audio})

    url = 'http://api.xfyun.cn/v1/service/v1/iat'
    api_key = '678d5046984ffcbb5ab0bff0df91ba53'
    param = {"engine_type": "sms16k", "aue": "raw"}

    x_appid = '5bc8447e'
    x_param = base64.b64encode(json.dumps(param).replace(' ', ''))
    x_time = int(int(round(time.time() * 1000)) / 1000)
    x_checksum = hashlib.md5(api_key + str(x_time) + x_param).hexdigest()
    x_header = {'X-Appid': x_appid,
                'X-CurTime': x_time,
                'X-Param': x_param,
                'X-CheckSum': x_checksum,
                'Content-Type':'application/x-www-form-urlencoded'}
    http=urllib3.PoolManager()
    result = http.request('POST',url, body=body, headers=x_header).data.decode()
    print result
    json_obj=json.loads(result)
    return json_obj['data']

def main():
    logging.basicConfig(level=logging.INFO)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    rospy.init_node('kdxf_asr', anonymous=True)
    while not rospy.is_shutdown():
        data = mic.listen()
        text = stt(data)
        hello_str = "REC: %s" % text
        rospy.loginfo(hello_str)
        if text:
            pub.publish(text)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
