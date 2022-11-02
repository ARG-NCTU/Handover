from arg_robotics_tools import websocket_rosbridge as socket
import time
from datetime import datetime
import numpy as np
import statistics

delay_time_1000 = []
period=0.001
t=time.time()

def subscriber_callback(message):
    global delay_time_1000
    now = datetime.now()
    delay_time_1000.append((now.timestamp() - message['linear']['x']))
    if(len(delay_time_1000) == 100000):
        print("mean RTT", sum(delay_time_1000)/100000)
        print("max RTT", max(delay_time_1000))
        print("min RTT", min(delay_time_1000))
        print("STD RTT", statistics.stdev(delay_time_1000))
        delay_time_1000.clear()

easy_subscriber = socket.ros_socket('10.8.0.3', 9090)
easy_subscriber.subscriber('/subscribe_topic', subscriber_callback , 1)

while True:
    t+=period
    time.sleep(max(0,t-time.time()))