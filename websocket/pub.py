from arg_robotics_tools import websocket_rosbridge as socket
import time
from datetime import datetime

easy_publisher = socket.ros_socket('192.168.0.185', 9090)

data = {'data':0}
talker = easy_publisher.publisher('/publish_topic', 'std_msgs/Float64')
period=0.001
t=time.time()

while True:
    now = datetime.now()
    data['data'] = now.timestamp()
    easy_publisher.pub(talker, data)
    t+=period
    time.sleep(max(0,t-time.time()))
