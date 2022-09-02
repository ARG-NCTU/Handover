#!/usr/bin/env python3

from re import sub
import rospy
import time
from handover_grasping.utils import get_pcd_left, get_pcd_right, get_view, width_detect
from .utils import Affordance_predict
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Int16
from geometry_msgs.msg import WrenchStamped, Pose
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf import TransformListener, TransformerROS
import rospy
import warnings
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib_msgs.msg import *
from vx300s_bringup.srv import *
from handover.srv import *
import yaml
warnings.filterwarnings("ignore")

# Create a trivial action server
class HandoverServer:
    def __init__(self, name, mode, arm):
        self._sas = SimpleActionServer(name, TestAction, execute_cb=self.execute_cb, auto_start=False)
        self._sas.start()
        self.r = TriggerRequest()
        info = rospy.wait_for_message('camera_right/color/camera_info', CameraInfo)
        with open('./config/multi_view.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
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
        self.cl_count = 0
        self.c = 0
        self.dis = None
        self.aff_map = None
        self.pred = Affordance_predict(self.arm, info.P[0], info.P[5], info.P[2], info.P[6])
        self.VIEW = None
        self.led_state = 0
        
        # Subscriber:
        self.force = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.callback_force_msgs)

        self.color_sub_r = message_filters.Subscriber('/camera_right/color/image_raw/compressed', CompressedImage)
        self.depth_sub_r = message_filters.Subscriber('/camera_right/aligned_depth_to_color/image_raw', Image)
        self.color_sub_l = message_filters.Subscriber('/camera_left/color/image_raw/compressed', CompressedImage)
        self.depth_sub_l = message_filters.Subscriber('/camera_left/aligned_depth_to_color/image_raw', Image)
        self.color_sub_m = message_filters.Subscriber('/camera_mid/color/image_raw/compressed', CompressedImage)
        self.depth_sub_m = message_filters.Subscriber('/camera_mid/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([self.color_sub_r, self.depth_sub_r, self.color_sub_l, self.depth_sub_l, self.color_sub_m, self.depth_sub_m], 5, 5)
        ts.registerCallback(self.callback_img_msgs)

        # Publisher prediction image
        self.aff_pub_90 = rospy.Publisher("~affordance_map_90", Image, queue_size=10)
        self.aff_pub_0 = rospy.Publisher("~affordance_map_0", Image, queue_size=10)
        self.aff_pub_67 = rospy.Publisher("~affordance_map_67", Image, queue_size=10)
        self.aff_pub_22 = rospy.Publisher("~affordance_map_22", Image, queue_size=10)
        self.aff_pub_45 = rospy.Publisher("~affordance_map_45", Image, queue_size=10)
        self.aff_pub_center_pcl = rospy.Publisher("~affordance_map", Image, queue_size=10)
        self.pose_pub_right = rospy.Publisher("~pose_right", Pose, queue_size=10)
        self.pose_pub_left = rospy.Publisher("~pose_left", Pose, queue_size=10)

        self.angle =None

        self.LED_state = rospy.Publisher("~led_state", Int16, queue_size=10)

        # Service
        rospy.Service('~switch_loop', Trigger, self.switch_loop)

        self.go_45()

        rospy.loginfo("Server Initial Complete")


        """
        goal = 0 : Init
             = 1 : Multi-view Detect
             = 2 : Go target
             = 3 : Grasp and back
             = 4 : Check_dis
             = 5 : Wait_object
             = 6 : Single detect
             = 7 : User actively give
        """

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

    def go_90(self):
        try:
            go_pose = rospy.ServiceProxy("/{0}/multi_view_90".format('right_arm'), Trigger)
            resp = go_pose(self.r)
            return True
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))
            return True

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


    def generate_view(self, Cl, Dl, CR, DR, id):
        pcl_l, _ = get_pcd_left(Cl, Dl, self.config['rotation_matrix']['rotation_matrix_left'])
        pcl_r, _ = get_pcd_right(CR, DR)

        image, depth = get_view(pcl_r, pcl_l, self.config['multi_view']['front'], self.config['multi_view']['lookat'], self.config['multi_view']['up'], self.config['multi_view']['zoom'],id)

        return image, depth

    def single_pred(self, c, d, armm):
        target1, _, dis1, aff_map1, v1, a, _ = self.pred.predict(c, d, armm)
        return target1, aff_map1, dis1, a

    def multi_pred(self, c90, d90, c67, d67, c45, d45, c22, d22, c0, d0):
        aff_map90, v90, a90, center90 = self.pred.predict(c90, d90, 'right_arm', True)
        aff_map67, v67, a67, center67 = self.pred.predict(c67, d67, 'right_arm', True)
        aff_map45, v45, a45, center45 = self.pred.predict(c45, d45, 'right_arm', True)
        aff_map22, v22, a22, center22 = self.pred.predict(c22, d22, 'right_arm', True)
        aff_map0, v0, a0, center0 = self.pred.predict(c0, d0, 'right_arm', True)

        self.aff_pub_90.publish(aff_map90)
        self.aff_pub_67.publish(aff_map67)
        self.aff_pub_45.publish(aff_map45)
        self.aff_pub_22.publish(aff_map22)
        self.aff_pub_0.publish(aff_map0)

        dif90 = width_detect(d90, center90,a90)
        dif67 = width_detect(d67, center67, a67)
        dif45 = width_detect(d45, center45, a45)
        dif22 = width_detect(d22, center22, a22)
        dif0 = width_detect(d0, center0, a0)

        # 0, 1, 2, 3, 4
        DIF = [dif90, dif67, dif45, dif22, dif0]
        print(DIF)
        view_id = DIF.index(min(DIF))

        return view_id

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
    
    def callback_img_msgs(self, color_msg_r, depth_msg_r, color_msg_l, depth_msg_l, color_msg_m, depth_msg_m):
        self.color_r = color_msg_r
        self.depth_r = depth_msg_r
        self.color_l = color_msg_l
        self.depth_l = depth_msg_l
        self.color_m = color_msg_m
        self.depth_m = depth_msg_m

    def callback_force_msgs(self, msg):
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
            self.cl_count = 0
            action_1 = self.go_45()

            # open gripper
            action_2 = self.open_gripper()

            if action_1 == True and action_2 == True:
                self._sas.set_succeeded()
            else:
                self._sas.set_succeeded()

        # Multi-view Detect
        elif msg.goal == 1:
            rospy.loginfo('Multi-view detecting......')
            self.arm = 'right_arm'
            action = True

            cv_90, d_90 = self.msg2cv(self.color_m, self.depth_m)
            cv_45, d_45 = self.msg2cv(self.color_r, self.depth_r)
            cv_0, d_0 = self.msg2cv(self.color_l, self.depth_l)

            cv_67, d_67 = self.generate_view(cv_0, d_0, cv_90, d_90, 1)
            cv_22, d_22 = self.generate_view(cv_0, d_0, cv_90, d_90, 4)

            self.VIEW = self.multi_pred(cv_90, d_90, cv_67, d_67, cv_45, d_45, cv_22, d_22, cv_0, d_0)

            if self.VIEW == 0:
                # 90
                action = self.go_90()
            elif self.VIEW == 1:
                # 67
                action = self.go_67()
            elif self.VIEW == 2:
                # 45
                action = self.go_45()
            elif self.VIEW == 3:
                # 22
                action = self.go_22()
            elif self.VIEW == 4:
                # 0
                action = self.go_0()

            time.sleep(0.5)

            c, d = self.msg2cv(self.color_r, self.depth_r)
            self.target, aff_map, self.dis, self.angle = self.single_pred(c, d,'right_arm')

            self.aff_pub_center_pcl.publish(aff_map)

            if self.target != None and action == True:
                self._sas.set_succeeded()
            else:
                _ = self.go_45()
                self._sas.set_aborted()

        # Go target
        elif msg.goal == 2:
            start = self.get_pose()
            rospy.loginfo('Go Target...')

            # ---------------- Waypoint ---------------- 
            listener = TransformListener()
            transformer = TransformerROS()
            listener.waitForTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0))
            c_pose = ee_poseRequest()
            c_pose.target_pose.position.x = trans[0]
            c_pose.target_pose.position.y = trans[1]
            c_pose.target_pose.position.z = trans[2]

            c_pose_list = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
            targetpose_list = [self.target.target_pose.position.x, self.target.target_pose.position.y, self.target.target_pose.position.z, self.target.target_pose.orientation.x, self.target.target_pose.orientation.y, self.target.target_pose.orientation.z, self.target.target_pose.orientation.w]
            waypoint_list = waypoint(c_pose_list, targetpose_list)

            for sub_pose in waypoint_list:
                goal_pose = ee_poseRequest()
                goal_pose.target_pose.position.x = sub_pose[0]
                goal_pose.target_pose.position.y = sub_pose[1]
                goal_pose.target_pose.position.z = sub_pose[2]
                goal_pose.target_pose.orientation.x = sub_pose[3]
                goal_pose.target_pose.orientation.y = sub_pose[4]
                goal_pose.target_pose.orientation.z = sub_pose[5]
                goal_pose.target_pose.orientation.w = sub_pose[6]

                self.target = go_pose

                action = self.go_target()
            # ---------------- Waypoint ----------------

            # ---------------- Waypoint ---------------- 
            # listener = TransformListener()
            # transformer = TransformerROS()
            # listener.waitForTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
            # (trans, rot) = listener.lookupTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0))
            # c_pose = ee_poseRequest()
            # c_pose.target_pose.position.x = trans[0]
            # c_pose.target_pose.position.y = trans[1]
            # c_pose.target_pose.position.z = trans[2]

            # c_pose_list = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
            # targetpose_list = [self.target.target_pose.position.x, self.target.target_pose.position.y, self.target.target_pose.position.z, self.target.target_pose.orientation.x, self.target.target_pose.orientation.y, self.target.target_pose.orientation.z, self.target.target_pose.orientation.w]
            # waypoint_list = waypoint(c_pose_list, targetpose_list)

            # for sub_pose in waypoint_list:
            #     goal_pose = ee_poseRequest()
            #     goal_pose.target_pose.position.x = sub_pose[0]
            #     goal_pose.target_pose.position.y = sub_pose[1]
            #     goal_pose.target_pose.position.z = sub_pose[2]
            #     goal_pose.target_pose.orientation.x = sub_pose[3]
            #     goal_pose.target_pose.orientation.y = sub_pose[4]
            #     goal_pose.target_pose.orientation.z = sub_pose[5]
            #     goal_pose.target_pose.orientation.w = sub_pose[6]

            #     self.target = go_pose

            #     action = self.go_target()
            # ---------------- Waypoint ----------------

            stop = self.get_pose()

            if round(stop.target_pose.position.x,2) == round(start.target_pose.position.x,2):
                self.count += 1
                self._sas.set_aborted()
            else:
                self.count += 1
                self.cl_count += 1
                self._sas.set_succeeded()

        # Grasp and back
        elif msg.goal == 3:
            rospy.loginfo('Grasp and back...')
            action_1 = True
            action_2 = True
            action_3 = True

            #Grasp
            rospy.sleep(1.5)
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
            rospy.loginfo('Check dis...')
            depth_list = []
            c = 0
            for _ in range(8):
                cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
                _, _, dis, _ = self.single_pred(cv_r, d_r, self.arm)
                if dis > 0.0:
                    depth_list.append(dis)
                    c += 1
            if c != 0:
                self.dis = sum(depth_list)/c
            else:
                self.dis = 0.0

            if self.cl_count == 2:
                # self.pred.switch()
                self._sas.set_succeeded()
            elif self.cl_count == 1:
                self.pred.switch()
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
                    break

            self._sas.set_succeeded()

        # Single detect
        elif msg.goal == 6:
            rospy.loginfo('single detecting...')
            cv_r, d_r = self.msg2cv(self.color_r, self.depth_r)
            self.target, affmapp, self.dis, _ = self.single_pred(cv_r, d_r, 'right_arm')
            self.aff_pub_center_pcl.publish(affmapp)

            if self.target != None:
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()

        # User actively give
        elif msg.goal == 7:
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

            rospy.sleep(1.5)
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
    rospy.init_node('handover_server')
    server = HandoverServer(name='handover_action', mode='handover', arm='right_arm')
    rospy.on_shutdown(server.onShutdown)
    rospy.spin()