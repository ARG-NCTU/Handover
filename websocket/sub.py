from arg_robotics_tools import websocket_rosbridge as socket
import time
import numpy as np
import pandas as pd
import statistics

delay_time_100000 = []
period=0.001
t=time.time()

def subscriber_callback(message):
    global delay_time_100000
    # print(message['linear']['x'])
    delay = time.time_ns() * 1e-09 - message['linear']['x']
    delay_time_100000.append(delay)
    if(len(delay_time_100000) == 100000.0):
        print("mean RTT", sum(delay_time_100000)/100000.0)
        print("max RTT", max(delay_time_100000))
        print("min RTT", min(delay_time_100000))
        print("STD RTT", statistics.stdev(delay_time_100000))
        dict = {'raw': delay_time_100000, 
                'mean':(sum(delay_time_100000)/100000.0), 
                'max':max(delay_time_100000), 
                'min':min(delay_time_100000),
                'STD':statistics.stdev(delay_time_100000)}
        df = pd.DataFrame(dict)
        df.to_csv('eth_ctl.csv')
        delay_time_100000.clear()

easy_subscriber = socket.ros_socket('192.168.0.185', 9090)
easy_subscriber.subscriber('/subscribe_topic', subscriber_callback, 1)

while True:
    t+=period
    time.sleep(max(0,t-time.time()))