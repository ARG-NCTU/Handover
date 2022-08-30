#!/usr/bin/env python3

from re import sub
import rospy
import time
import sys
import argparse
import cv2
import random
import numpy as np
sys.path.append('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/ros_handover/handover/src/HANet')
from HANet.utils import Affordance_predict, get_pcd_left, get_pcd_right, get_view, waypoint, width_detect
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import WrenchStamped, Pose
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf import TransformListener, TransformerROS, transformations
import rospy
from std_msgs.msg import Int16
import warnings
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib_msgs.msg import *
from vx300s_bringup.srv import *
from handover.srv import *
# from arg_tools.utils import waypoint
warnings.filterwarnings("ignore")

# Create a trivial action server
class HandoverServer:
    def __init__(self, name, mode, arm):
        self._sas = SimpleActionServer(name, TestAction, execute_cb=self.execute_cb, auto_start=False)
        self._sas.start()
        self.r = TriggerRequest()
        info = rospy.wait_for_message('camera_right/color/camera_info', CameraInfo)
        self.id = 1000
        self.bridge = CvBridge()
        self.mode = mode
        self.arm = arm
        self.color_r = None
        self.depth_r = None
        self.color_l = None
        self.depth_l = None
        self.color_m = None
        self.depth_m = None
        self.target = None
        self.model_on = False
        self.f_x = 0
        self.f_y = 0
        self.f_z = 0
        self.count = 0
        self.c = 0
        self.dis = None
        self.aff_map = None
        self.pred = Affordance_predict(self.arm, info.P[0], info.P[5], info.P[2], info.P[6], self.mode)
        self.VIEW = None
        self.angle =None
        # self.pred.factor = 1
        
        # Subscriber:
        self.force = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.callback_force_msgs)

        self.color_sub_r = message_filters.Subscriber('/camera_right/color/image_raw/compressed', CompressedImage)
        self.depth_sub_r = message_filters.Subscriber('/camera_right/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([self.color_sub_r, self.depth_sub_r], 5, 5)
        ts.registerCallback(self.callback_img_msgs)

        # Publisher prediction image
        self.aff_pub_90 = rospy.Publisher("~affordance_map_90", Image, queue_size=10)
        self.aff_pub_0 = rospy.Publisher("~affordance_map_0", Image, queue_size=10)
        self.aff_pub_67 = rospy.Publisher("~affordance_map_67", Image, queue_size=10)
        self.aff_pub_22 = rospy.Publisher("~affordance_map_22", Image, queue_size=10)
        self.aff_pub_45 = rospy.Publisher("~affordance_map_45", Image, queue_size=10)
        self.aff_pub_center_pcl = rospy.Publisher("~affordance_map_center_pcl", Image, queue_size=10)
        self.pose_pub_right = rospy.Publisher("~pose_right", Pose, queue_size=10)
        self.pose_pub_left = rospy.Publisher("~pose_left", Pose, queue_size=10)
        self.LED_state = rospy.Publisher("~led_state", Int16, queue_size=10)

        # Service
        rospy.Service('~switch_loop', Trigger, self.switch_loop)
        rospy.Service('~view', int_command, self.set_view)

        self.go_45()

        MSG = Int16()
        MSG.data = 0
        self.LED_state.publish(MSG)

        rospy.loginfo("Server Initial Complete")

        """
        goal = 0 : Init
             = 1 : Multi-view Detect
             = 2 : Go target
             = 3 : Grasp and back
             = 4 : Check_dis
             = 5 : Wait_object
             = 6 : Single detect
        """

    def go_90(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/multi_view_90".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def go_67(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/multi_view_67".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def go_45(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/go_handover".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def go_22(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/multi_view_22".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def go_0(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/multi_view_0".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def open_gripper(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/gripper_open".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def close_gripper(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/gripper_close".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def go_target(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/go_pose".format(self.arm), ee_pose)
            resp = go_pose(self.target)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return False

    def set_view(self, req):
        res = int_commandResponse()
        res.result = 'success'
        self.VIEW = req.view.data

        return res

    def multi_view(self):
        # 90
        self.go_90()
        rospy.sleep(0.5)
        cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
        _, _, _, angle, center = self.single_pred(cv_r, d_r, 'right_arm')
        dif90 = width_detect(d_r, center, 0)
        self.go_45()
        rospy.sleep(0.5)
        # 67
        self.go_67()
        rospy.sleep(0.5)
        cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
        _, _, _, angle, center = self.single_pred(cv_r, d_r, 'right_arm')
        dif67 = width_detect(d_r, center, 0)
        self.go_45()
        rospy.sleep(0.5)

        # 45
        cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
        _, _, _, angle, center = self.single_pred(cv_r, d_r, 'right_arm')
        dif45 = width_detect(d_r, center, 0)
        rospy.sleep(0.5)
        # 22
        self.go_22()
        rospy.sleep(0.5)
        cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
        _, _, _, angle, center = self.single_pred(cv_r, d_r, 'right_arm')
        dif22 = width_detect(d_r, center, 0)
        self.go_45()
        rospy.sleep(0.5)
        # 0
        self.go_0()
        rospy.sleep(0.5)
        cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
        _, _, _, angle, center = self.single_pred(cv_r, d_r, 'right_arm')
        dif0 = width_detect(d_r, center, 0)
        self.go_45()

        DIF = [dif90, dif67, dif45, dif22, dif0]
        DIF.index(min(DIF))

        return DIF.index(min(DIF))


    def single_pred(self, c, d, armm):
        target1, _, dis1, aff_map1, v1, a, c = self.pred.predict_mul(c, d, armm)
        return target1, aff_map1, dis1, a, c


    def msg2cv(self, c, d):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(c, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(d, "16UC1")

        except CvBridgeError as e:
            print(e)
            return

        return cv_image, cv_depth

    def switch_loop(self, req):
        res = TriggerResponse()
        self.pred.switch()
        res.success = True

        return res

    def get_pose(self):
        listener = TransformListener()
        listener.waitForTransform('right_arm'+'/base_link', 'right_arm'+'/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('right_arm'+'/base_link', 'right_arm'+'/ee_arm_link', rospy.Time(0))
        
        c_pose = ee_poseRequest()
        c_pose.target_pose.position.x = trans[0]
        c_pose.target_pose.position.y = trans[1]
        c_pose.target_pose.position.z = trans[2]
        c_pose.target_pose.orientation.x = rot[0]
        c_pose.target_pose.orientation.y = rot[1]
        c_pose.target_pose.orientation.z = rot[2]
        c_pose.target_pose.orientation.w = rot[3]

        return c_pose
    
    def callback_img_msgs(self, color_msg_r, depth_msg_r):
        self.color_r = color_msg_r
        self.depth_r = depth_msg_r

    def callback_force_msgs(self, msg):
        # self.f_x = int(msg.wrench.force.x)
        # self.f_y = int(msg.wrench.force.y)
        # self.f_z = int(msg.wrench.force.z)
        self.f_x = round(msg.wrench.force.x, 3)
        self.f_y = round(msg.wrench.force.y, 3)
        self.f_z = round(msg.wrench.force.z, 3)

    def execute_cb(self, msg):
        # Init
        if msg.goal == 0:
            MSG = Int16()
            MSG.data = 1
            self.LED_state.publish(MSG)
            action_1 = True
            action_2 = True
            # Go initial pose
            self.count = 0
            action_1 = self.go_45()

            # open gripper
            action_2 = self.open_gripper()

            if action_1 == True and action_2 == True:
                self._sas.set_succeeded()
            else:
                self._sas.set_succeeded()

        # Multi-view Detect
        elif msg.goal == 1:
            self.arm = 'right_arm'
            action = True

            view_id = self.multi_view()
            

            if self.VIEW < 10:
                view_id = self.VIEW

            if view_id == 0:
                # 90
                action = self.go_90()
            elif view_id == 1:
                # 67
                action = self.go_67()
            elif view_id == 2:
                # 45
                action = self.go_45()
            elif view_id == 3:
                # 22
                action = self.go_22()
            elif view_id == 4:
                # 0
                action = self.go_0()

            time.sleep(0.5)

            c, d = self.msg2cv(self.color_r, self.depth_r)
            self.target, aff_map, self.dis, self.angle, _ = self.single_pred(c, d,'right_arm')
            self.aff_pub_center_pcl.publish(aff_map)

            if self.target != None:
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()
                

        # Go target
        elif msg.goal == 2:
            start = self.get_pose()
            rospy.loginfo('Go Target...')
            # print(self.target)
            action = self.go_target()

            stop = self.get_pose()

            self.count += 1

            if round(stop.target_pose.position.x,2) == round(start.target_pose.position.x,2):
                self._sas.set_aborted()
            else:
                self._sas.set_succeeded()


        # Grasp and back
        elif msg.goal == 3:
            action_1 = True
            action_2 = True
            action_3 = True

            rospy.sleep(1.5)

            #Grasp
            action_1 = self.close_gripper()

            rospy.sleep(0.5)

            # Back
            action_2 = self.go_45()
            action_3 = self.open_gripper()
            
            if action_1 == True and action_2 == True and action_3 == True:
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()

            MSG = Int16()
            MSG.data = 0
            self.LED_state.publish(MSG)

        # Check distance
        elif msg.goal == 4:
            cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
            _, _, self.dis, _, _ = self.single_pred(cv_r, d_r, self.arm)

            rospy.loginfo(str(self.dis))
            if self.count == 2:
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()

            rospy.sleep(0.5)

        # Wait object
        elif msg.goal == 5:
            MSG = Int16()
            MSG.data = 2
            self.LED_state.publish(MSG)
            rospy.sleep(1)
            x = self.f_x
            while True:
                if abs(x-self.f_x) > 1:
                # if x != self.f_x:
                    print(x, self.f_x)
                    break

            self._sas.set_succeeded()

        # Single detect
        elif msg.goal == 6:
            cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
            self.target, affmapp, self.dis, self.angle, _ = self.single_pred(cv_r, d_r, 'right_arm')
            self.aff_pub_center_pcl.publish(affmapp)

            if self.target != None:
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()

        elif msg.goal == 7:
            print('kkkkkkkkkkkkkk',self.angle)
            for _ in range(2-self.count):
                try:
                    go_pose = rospy.ServiceProxy("/{0}/forward".format('right_arm'), Trigger)
                    resp = go_pose(self.r)
                except rospy.ServiceException as exc:
                    print("service did not process request: " + str(exc))

            if self.VIEW != 0 and self.VIEW != 4:
                if self.angle == 90:
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turnto90".format('right_arm'), Trigger)
                        resp = go_pose(self.r)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == -45:
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turnto45".format('right_arm'), Trigger)
                        resp = go_pose(self.r)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == 45:
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turnto_45".format('right_arm'), Trigger)
                        resp = go_pose(self.r)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))

            elif self.VIEW == 0:
                f = float_commandRequest()
                if self.angle == 90:
                    f.view.data = -2.4
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == -45:
                    f.view.data = 1.5
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == 45:
                    f.view.data = -0.1
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
            elif self.VIEW == 4:
                print('TURN')
                f = float_commandRequest()
                if self.angle == 90:
                    f.view.data = 2.7
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == -45:
                    f.view.data = 0.4
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))
                elif self.angle == 45:
                    f.view.data = 1.8
                    try:
                        go_pose = rospy.ServiceProxy("/{0}/turn".format('right_arm'), float_command)
                        resp = go_pose(f)
                    except rospy.ServiceException as exc:
                        print("service did not process request: " + str(exc))

            self._sas.set_succeeded()



    def onShutdown(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/go_sleep".format('right_arm'), Trigger)
            resp = go_pose(self.r)
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))

        rospy.sleep(0.5)
        rospy.loginfo("Shutdown.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Set up')
    parser.add_argument('--mode', type=str, default = 'handover')
    parser.add_argument('--arm', type=str, default = 'right_arm')
    args = parser.parse_args()

    rospy.init_node('handover_server')
    server = HandoverServer(name='handover_action', mode=args.mode, arm=args.arm)
    rospy.on_shutdown(server.onShutdown)
    rospy.spin()