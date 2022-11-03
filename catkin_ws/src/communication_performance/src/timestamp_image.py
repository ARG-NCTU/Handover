#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Int32, Int16, Float64
import time
import statistics
import pandas as pd

class timestamp_test():
    def __init__(self):
        rospy.Subscriber('/publish_topic', Twist, self.timestamp_callback, queue_size=60)
        self.delay_time_3000 = []
        self.i = 0

        # self.timestamp_msg = TwistStamped()
        
    def timestamp_callback(self, msg):
        delay = time.time_ns() * 1e-09 - msg.linear.x
        self.delay_time_3000.append(delay)
        # print(self.delay_time_3000)
        self.i += 1
        if(self.i == 3000):
            self.i = 0
            print("mean RTT", sum(self.delay_time_3000)/3000)
            print("max RTT", max(self.delay_time_3000))
            print("min RTT", min(self.delay_time_3000))
            print("STD RTT", statistics.stdev(self.delay_time_3000))
            dict = {'raw': self.delay_time_3000, 
                'mean':(sum(self.delay_time_3000)/3000), 
                'max':max(self.delay_time_3000), 
                'min':min(self.delay_time_3000),
                'STD':statistics.stdev(self.delay_time_3000)}
            df = pd.DataFrame(dict)
            df.to_csv('5g_ctl.csv')
            self.delay_time_3000. clear()

if __name__=='__main__':
    rospy.init_node("pyhton_demo", anonymous=False)
    test = timestamp_test()

    rospy.spin()
