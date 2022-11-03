from arg_robotics_tools import websocket_rosbridge as socket
import time
from datetime import datetime
import numpy as np

period=0.001
t=time.time()

class PSub_image():
    def __init__(self):
        self.easy_socket = socket.ros_socket('10.8.0.3', 9090)
        self.data = {'linear':{'x':0.0, 'y':0.0, 'z':0.0}, 'angular':{'x':0.0, 'y':0.0, 'z':0.0}}
        self.talker = self.easy_socket.publisher('/publish_topic', 'geometry_msgs/Twist')
        self.easy_socket.subscriber('/camera_mid/color/image_raw/compressed', self.sub_callback , 1)
        self.easy_socket.subscriber('/camera_left/color/image_raw/compressed', self.sub_callback , 1)
        self.easy_socket.subscriber('/camera_right/color/image_raw/compressed', self.sub_callback , 1)
    def sub_callback(self, msg):
        # print(msg['header']['stamp'])
        self.data['linear']['x'] = msg['header']['stamp']['secs'] + msg['header']['stamp']['nsecs'] * 1e-09
        self.easy_socket.pub(self.talker, self.data)

test = PSub_image()

while True:
    t+=period
    time.sleep(max(0,t-time.time()))