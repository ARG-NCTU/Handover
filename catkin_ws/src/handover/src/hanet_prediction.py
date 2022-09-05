#!/usr/bin/env python3

import cv2
import rospy
import rospkg
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from handover_grasping.model import HANet
from scipy.spatial.transform import Rotation
from tf import TransformListener, TransformerROS, transformations
import tf
from .utils import processing
from vx300s_bringup.srv import *

class Affordance_predict_server():
    def __init__(self, arm, fx, fy, cx, cy, Mode):
        r = rospkg.RosPack()
        self.path = r.get_path("handover")
        self.net = HANet()
        self.Mode = Mode
        self.net.load_state_dict(torch.load(self.path+'/src/HANet/HANet.pth'))
        self.A = [90,45,0,-45]
        self.net = self.net.cuda()
        self.bridge = CvBridge()
        self.target_cam_dis = 1000
        self.listener = TransformListener()
        self.transformer = TransformerROS()
        self.value = None
        self.arm = arm
        self.factor = 0.5
        self.go_loop = False
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy


    def switch(self):
        if self.go_loop:
            rospy.loginfo("Enable Open Loop")
            self.go_loop = False
        else:
            rospy.loginfo("Enable Close Loop")
            self.go_loop = True
        

    def predict(self, cv_image, cv_depth, Arm, single=False):
        # Convert msg type
        cv_depth_grasp = cv_depth.copy()

        # Do prediction
        color_in, depth_in = processing(cv_image, cv_depth)
        color_in = color_in.cuda()
        depth_in = depth_in.cuda()

        # Get gripping point base on camera link
        x, y, aff_pub, angle = self.net.get_affordanceMap(color_in, depth_in, cv_depth_grasp)

        if single:
            aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
            p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")
            return p, self.value, angle, [int(y), int(x)]
        else:
            if x != 0 and y!=0:
                z = cv_depth_grasp[int(y), int(x)]/1000.0

                aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
                p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")

                camera_x, camera_y, camera_z = self.getXYZ(x, y, z)
                self.target_cam_dis = camera_z

                # Add to pose msgs
                Target_pose = ee_poseRequest()

                if self.Mode == 'handover':
                    rot = Rotation.from_euler('xyz', [angle, -15, 0], degrees=True)
                    # else:
                    Target_pose.target_pose.position.x = camera_x - 0.02
                    Target_pose.target_pose.position.y = camera_y
                    Target_pose.target_pose.position.z = camera_z + 0.05
                else:
                    rot = Rotation.from_euler('xyz', [0, 80, 0], degrees=True)
                    if Arm == 'right_arm':
                        Target_pose.target_pose.position.x = camera_x
                        Target_pose.target_pose.position.y = camera_y + 0.027
                        Target_pose.target_pose.position.z = camera_z - 0.01
                    else:
                        Target_pose.target_pose.position.x = camera_x
                        Target_pose.target_pose.position.y = camera_y - 0.04
                        Target_pose.target_pose.position.z = camera_z + 0.05

                rot_quat = rot.as_quat()

                Target_pose.target_pose.orientation.x = rot_quat[0]
                Target_pose.target_pose.orientation.y = rot_quat[1]
                Target_pose.target_pose.orientation.z = rot_quat[2]
                Target_pose.target_pose.orientation.w = rot_quat[3]

                target_pose, go_ok = self.camera2world(Target_pose, Arm)

                if z == 0.0:
                    z = 0.4

                if z == 0.0 or Target_pose.target_pose.position.x > 3.0:
                    go_ok = False
                    return None, False, self.target_cam_dis, None, 0, angle, [int(y), int(x)]
                print('111')
                return target_pose, go_ok, self.target_cam_dis, p, self.value, angle, [int(y), int(x)]
            else:
                print('222')
                return None, False, self.target_cam_dis, None, 0, angle, [int(y), int(x)]

    def camera2world(self, camera_pose, arm):
        vaild = True
        try:
            if arm == 'right_arm':
                self.listener.waitForTransform('right_arm/base_link', 'camera_right_link', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('right_arm/base_link', 'camera_right_link', rospy.Time(0))
            else:
                self.listener.waitForTransform('left_arm/base_link', 'camera_left_link', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('left_arm/base_link', 'camera_left_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error TF listening")
            return

        tf_pose = ee_poseRequest()

        pose = tf.transformations.quaternion_matrix(np.array(
                        [camera_pose.target_pose.orientation.x, camera_pose.target_pose.orientation.y, camera_pose.target_pose.orientation.z, camera_pose.target_pose.orientation.w]))

        pose[0, 3] = camera_pose.target_pose.position.x
        pose[1, 3] = camera_pose.target_pose.position.y
        pose[2, 3] = camera_pose.target_pose.position.z

        offset_to_world = np.matrix(transformations.quaternion_matrix(rot))
        offset_to_world[0, 3] = trans[0]
        offset_to_world[1, 3] = trans[1]
        offset_to_world[2, 3] = trans[2]

        tf_pose_matrix = np.array(np.dot(offset_to_world, pose))

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_matrix([[tf_pose_matrix[0, 0], tf_pose_matrix[0, 1], tf_pose_matrix[0, 2]], [tf_pose_matrix[1, 0], tf_pose_matrix[1, 1], tf_pose_matrix[1, 2]], [tf_pose_matrix[2, 0], tf_pose_matrix[2, 1], tf_pose_matrix[2, 2]]])

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        if tf_pose_matrix[0, 3] >= 0.15 and tf_pose_matrix[0, 3] <= 1.5:
            if arm == 'left_arm':
                # tf_pose.target_pose.position.x = tf_pose_matrix[0, 3] + 0.07
                tf_pose.target_pose.position.x = tf_pose_matrix[0, 3]
                tf_pose.target_pose.position.y = tf_pose_matrix[1, 3] + 0.05
                # tf_pose.target_pose.position.z = tf_pose_matrix[2, 3] - 0.07
                tf_pose.target_pose.position.z = tf_pose_matrix[2, 3]
            else:
                tf_pose.target_pose.position.x = tf_pose_matrix[0, 3]
                tf_pose.target_pose.position.y = tf_pose_matrix[1, 3]
                # tf_pose.target_pose.position.z = tf_pose_matrix[2, 3] - 0.07
                tf_pose.target_pose.position.z = tf_pose_matrix[2, 3]

            tf_pose.target_pose.orientation.x = rot_quat[0]
            tf_pose.target_pose.orientation.y = rot_quat[1]
            tf_pose.target_pose.orientation.z = rot_quat[2]
            tf_pose.target_pose.orientation.w = rot_quat[3]

        if self.go_loop:
            try:
                if arm == 'right_arm':
                    self.listener.waitForTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.listener.lookupTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0))
                else:
                    self.listener.waitForTransform('left_arm/base_link', 'left_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.listener.lookupTransform('left_arm/base_link', 'left_arm/ee_arm_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Error TF listening")
                return
            tf_pose.target_pose.position.x = (tf_pose.target_pose.position.x - trans[0])*self.factor + trans[0]
            tf_pose.target_pose.position.y = (tf_pose.target_pose.position.y - trans[1])*self.factor + trans[1]
            tf_pose.target_pose.position.z = (tf_pose.target_pose.position.z - trans[2])*self.factor + trans[2]


        return tf_pose, vaild

    def getXYZ(self, x, y, zc):
        x = float(x)
        y = float(y)
        zc = float(zc)
        inv_fx = 1.0/self.fx
        inv_fy = 1.0/self.fy
        x = (x - self.cx) * zc * inv_fx
        y = (y - self.cy) * zc * inv_fy
        z = zc

        return z, -1*x, -1*y

class Affordance_predict():
    def __init__(self, arm, fx, fy, cx, cy, Mode):
        r = rospkg.RosPack()
        self.path = r.get_path("handover")
        self.net = HANet()
        self.Mode = Mode
        self.net.load_state_dict(torch.load(self.path+'/src/HANet/HANet.pth'))
        self.A = [90,45,0,-45]
        self.net = self.net.cuda()
        self.bridge = CvBridge()
        self.target_cam_dis = 1000
        self.listener = TransformListener()
        self.transformer = TransformerROS()
        self.value = None
        self.arm = arm
        self.factor = 0.5
        self.go_loop = False
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy


    def switch(self):
        if self.go_loop:
            rospy.loginfo("Enable Open Loop")
            self.go_loop = False
        else:
            rospy.loginfo("Enable Close Loop")
            self.go_loop = True
        

    def predict(self, cv_image, cv_depth, Arm, single=False):
        # Convert msg type
        cv_depth_grasp = cv_depth.copy()

        # Do prediction
        color_in, depth_in = processing(cv_image, cv_depth)
        color_in = color_in.cuda()
        depth_in = depth_in.cuda()

        # Get gripping point base on camera link
        x, y, aff_pub, angle = self.net.get_affordanceMap(color_in, depth_in, cv_depth_grasp)

        if single:
            aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
            p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")
            return p, self.value, angle, [int(y), int(x)]
        else:
            if x != 0 and y!=0:
                z = cv_depth_grasp[int(y), int(x)]/1000.0

                aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
                p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")

                camera_x, camera_y, camera_z = self.getXYZ(x, y, z)
                self.target_cam_dis = camera_z

                # Add to pose msgs
                Target_pose = ee_poseRequest()

                if self.Mode == 'handover':
                    rot = Rotation.from_euler('xyz', [angle, -15, 0], degrees=True)
                    # else:
                    Target_pose.target_pose.position.x = camera_x - 0.02
                    Target_pose.target_pose.position.y = camera_y
                    Target_pose.target_pose.position.z = camera_z + 0.05
                else:
                    rot = Rotation.from_euler('xyz', [0, 80, 0], degrees=True)
                    if Arm == 'right_arm':
                        Target_pose.target_pose.position.x = camera_x
                        Target_pose.target_pose.position.y = camera_y + 0.027
                        Target_pose.target_pose.position.z = camera_z - 0.01
                    else:
                        Target_pose.target_pose.position.x = camera_x
                        Target_pose.target_pose.position.y = camera_y - 0.04
                        Target_pose.target_pose.position.z = camera_z + 0.05

                rot_quat = rot.as_quat()

                Target_pose.target_pose.orientation.x = rot_quat[0]
                Target_pose.target_pose.orientation.y = rot_quat[1]
                Target_pose.target_pose.orientation.z = rot_quat[2]
                Target_pose.target_pose.orientation.w = rot_quat[3]

                target_pose, go_ok = self.camera2world(Target_pose, Arm)

                if z == 0.0:
                    z = 0.4

                if z == 0.0 or Target_pose.target_pose.position.x > 3.0:
                    go_ok = False
                    return None, False, self.target_cam_dis, None, 0, angle, [int(y), int(x)]
                print('111')
                return target_pose, go_ok, self.target_cam_dis, p, self.value, angle, [int(y), int(x)]
            else:
                print('222')
                return None, False, self.target_cam_dis, None, 0, angle, [int(y), int(x)]

    def camera2world(self, camera_pose, arm):
        vaild = True
        try:
            if arm == 'right_arm':
                self.listener.waitForTransform('right_arm/base_link', 'camera_right_link', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('right_arm/base_link', 'camera_right_link', rospy.Time(0))
            else:
                self.listener.waitForTransform('left_arm/base_link', 'camera_left_link', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('left_arm/base_link', 'camera_left_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error TF listening")
            return

        tf_pose = ee_poseRequest()

        pose = tf.transformations.quaternion_matrix(np.array(
                        [camera_pose.target_pose.orientation.x, camera_pose.target_pose.orientation.y, camera_pose.target_pose.orientation.z, camera_pose.target_pose.orientation.w]))

        pose[0, 3] = camera_pose.target_pose.position.x
        pose[1, 3] = camera_pose.target_pose.position.y
        pose[2, 3] = camera_pose.target_pose.position.z

        offset_to_world = np.matrix(transformations.quaternion_matrix(rot))
        offset_to_world[0, 3] = trans[0]
        offset_to_world[1, 3] = trans[1]
        offset_to_world[2, 3] = trans[2]

        tf_pose_matrix = np.array(np.dot(offset_to_world, pose))

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_matrix([[tf_pose_matrix[0, 0], tf_pose_matrix[0, 1], tf_pose_matrix[0, 2]], [tf_pose_matrix[1, 0], tf_pose_matrix[1, 1], tf_pose_matrix[1, 2]], [tf_pose_matrix[2, 0], tf_pose_matrix[2, 1], tf_pose_matrix[2, 2]]])

        # Convert to quaternions and print
        rot_quat = rot.as_quat()

        if tf_pose_matrix[0, 3] >= 0.15 and tf_pose_matrix[0, 3] <= 1.5:
            if arm == 'left_arm':
                # tf_pose.target_pose.position.x = tf_pose_matrix[0, 3] + 0.07
                tf_pose.target_pose.position.x = tf_pose_matrix[0, 3]
                tf_pose.target_pose.position.y = tf_pose_matrix[1, 3] + 0.05
                # tf_pose.target_pose.position.z = tf_pose_matrix[2, 3] - 0.07
                tf_pose.target_pose.position.z = tf_pose_matrix[2, 3]
            else:
                tf_pose.target_pose.position.x = tf_pose_matrix[0, 3]
                tf_pose.target_pose.position.y = tf_pose_matrix[1, 3]
                # tf_pose.target_pose.position.z = tf_pose_matrix[2, 3] - 0.07
                tf_pose.target_pose.position.z = tf_pose_matrix[2, 3]

            tf_pose.target_pose.orientation.x = rot_quat[0]
            tf_pose.target_pose.orientation.y = rot_quat[1]
            tf_pose.target_pose.orientation.z = rot_quat[2]
            tf_pose.target_pose.orientation.w = rot_quat[3]

        if self.go_loop:
            try:
                if arm == 'right_arm':
                    self.listener.waitForTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.listener.lookupTransform('right_arm/base_link', 'right_arm/ee_arm_link', rospy.Time(0))
                else:
                    self.listener.waitForTransform('left_arm/base_link', 'left_arm/ee_arm_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.listener.lookupTransform('left_arm/base_link', 'left_arm/ee_arm_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Error TF listening")
                return
            tf_pose.target_pose.position.x = (tf_pose.target_pose.position.x - trans[0])*self.factor + trans[0]
            tf_pose.target_pose.position.y = (tf_pose.target_pose.position.y - trans[1])*self.factor + trans[1]
            tf_pose.target_pose.position.z = (tf_pose.target_pose.position.z - trans[2])*self.factor + trans[2]


        return tf_pose, vaild

    def getXYZ(self, x, y, zc):
        x = float(x)
        y = float(y)
        zc = float(zc)
        inv_fx = 1.0/self.fx
        inv_fy = 1.0/self.fy
        x = (x - self.cx) * zc * inv_fx
        y = (y - self.cy) * zc * inv_fy
        z = zc

        return z, -1*x, -1*y