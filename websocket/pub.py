from arg_robotics_tools import websocket_rosbridge as socket
import time

import numpy as np

easy_publisher = socket.ros_socket('192.168.0.185', 9090)

data = {'linear':{'x':0.0, 'y':0.0, 'z':0.0},
        'angular':{'x':0.0, 'y':0.0, 'z':0.0}}
talker = easy_publisher.publisher('/publish_topic', 'geometry_msgs/Twist', 1)
period=0.001
t=time.time()

while True:
    print(time.time_ns() * 1e-09)
    data['linear']['x'] = time.time_ns() * 1e-09
    easy_publisher.pub(talker, data)
    t+=period
    time.sleep(max(0,t-time.time()))
