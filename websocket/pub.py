from arg_robotics_tools import websocket_rosbridge as socket
import time
from datetime import datetime
import numpy as np

easy_publisher = socket.ros_socket('10.8.0.3', 9090)

data = {'linear':{'x':0.0, 'y':0.0, 'z':0.0},
        'angular':{'x':0.0, 'y':0.0, 'z':0.0}}
talker = easy_publisher.publisher('/publish_topic', 'std_msgs/Twist')
period=0.001
t=time.time()

while True:
    now = datetime.now()
    data['linear']['x'] = now.timestamp()
    easy_publisher.pub(talker, data)
    t+=period
    time.sleep(max(0,t-time.time()))
