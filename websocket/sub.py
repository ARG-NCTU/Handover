from arg_robotics_tools import websocket_rosbridge as socket
import time
from datetime import datetime

data = {'data':0}
delay_time_1000 = 0
i = 0
period=0.001
t=time.time()

def subscriber_callback(message):
    global i, delay_time_1000
    i += 1
    now = datetime.now()
    # delay_time = now.timestamp() - message['data']
    delay_time_1000 += (now.timestamp() - message['data'])
    if(i == 1000):
        i = 0
        print("RTT", (delay_time_1000 / 1000))
        delay_time_1000 = 0

easy_subscriber = socket.ros_socket('192.168.0.185', 9090)
easy_subscriber.subscriber('/subscribe_topic', subscriber_callback , 1)

while True:
    t+=period
    time.sleep(max(0,t-time.time()))