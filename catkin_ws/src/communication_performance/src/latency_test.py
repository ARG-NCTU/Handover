#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Int32, Int16, Float64

class timestamp_test():
    def __init__(self):
        rospy.Subscriber('/publish_topic', Float64, self.timestamp_callback, queue_size=1000)
        self.timestamp_pub = rospy.Publisher('/subscribe_topic', Float64, queue_size=1000)

        # self.timestamp_msg = TwistStamped()

    def timestamp_callback(self, msg):
        self.timestamp_pub.publish(msg)

if __name__=='__main__':
    rospy.init_node("pyhton_demo", anonymous=False)
    test = timestamp_test()

    rospy.spin()
