#!/usr/bin/env python3

import os
import sys
# from .model import HANet
# from .sn2pose import SurfaceNormal2Quaternion
import numpy as np
import math
import cv2
import rospy
from scipy import ndimage
import torch
import rospkg
from torchvision import  transforms
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from scipy.spatial.transform import Rotation
from tf import TransformListener, TransformerROS, transformations
import tf
from vx300s_bringup.srv import *
import open3d as o3d
import torch
import torch.nn as nn
import copy
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
from handover_grasping.model import HANet

path = os.getcwd()

image_net_mean = np.array([0.485, 0.456, 0.406])
image_net_std  = np.array([0.229, 0.224, 0.225])
transform = transforms.Compose([
                        transforms.ToTensor(),
                    ])

view_list = [[0.035383036947178384, -0.11473520506707088, -0.99276576966302177],[0.33174556112878911, -0.096479639115394977, -0.93842237926613203], [0.64792867802836396, -0.09877952620397483, -0.75526884841845232], [0.84069623781529446, -0.057016937367817905, -0.53849689374814824], [0.99861447483575372, 0.0095433475409705988, -0.051749929142214289]]
view_lookat = [[0.038923033325571767, 0.11577049089809685, 0.95233340037115533],[-0.015091367119826041, 0.11337003221324685, 0.93348533942550582], [-0.082389701902882792, 0.10829077301769802, 0.88378544822574601], [-0.082389701902882792, 0.10829077301769802, 0.88378544822574601], [-0.096112757113331185, 0.092377226951892391, 0.75000254321344495]]
view_up = [[0.029754411549334947, -0.99282647579126126, 0.11580269410189503],[0.030582102574529486, -0.99313381172004922, 0.11291575187070634], [0.011873286190625304, -0.99012528507193198, 0.13968158409848136], [0.01802913542122616, -0.990942704008271, 0.13306955943693297], [0.016047446312962718, -0.99180293091586658, 0.12676523850618271]]
zoom_list = [0.69999999999999978, 0.69999999999999987, 0.69999999999999987, 0.69999999999999987, 0.69999999999999987]


def processing(color, depth):
    color = cv2.resize(color, (224, 224))
    depth = cv2.resize(depth, (224, 224))
    # depth[depth > 1000] = 0.0

    color= color[:,:,[2,1,0]]
    color = (color/255.).astype(float)
    color_rgb = np.zeros(color.shape)
    # depth[depth > 1000] = 0

    for i in range(3):
        color_rgb[:, :, i] = (color[:, :, 2-i]-image_net_mean[i])/image_net_std[i]

    depth = np.round((depth/np.max(depth))*255).astype('int').reshape(1,depth.shape[0],depth.shape[1])
    # depth[depth > 1000] = 0

    depth = (depth/1000.).astype(float) # to meters
    depth = np.clip(depth, 0.0, 1.2)
    depth_3c = np.zeros(color.shape)

    for i in range(3):
        depth_3c[:, :, i] = (depth[:, :]-image_net_mean[i])/image_net_std[i]

    c = transform(color_rgb)
    d = transform(depth_3c)
    c = torch.unsqueeze(c,0)
    d = torch.unsqueeze(d,0)

    return c.float(), d.float()

class Affordance_predict():
    def __init__(self, arm, fx, fy, cx, cy):
        r = rospkg.RosPack()
        self.path = r.get_path("handover")
        self.net = HANet(pretrained=True)
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
        aff_pub, x, y, angle = self.net.get_affordanceMap(color_in, depth_in, cv_depth_grasp)
        print(x, y, angle)
        aff_pub = cv2.addWeighted(cv_image,0.7,aff_pub, 0.3,0,dtype=cv2.CV_8UC3)
        # aff_pub = np.array(aff_pub, dtype=np.uint8)
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

                # if self.Mode == 'handover':
                rot = Rotation.from_euler('xyz', [angle, -15, 0], degrees=True)
                # else:
                Target_pose.target_pose.position.x = camera_x - 0.02
                Target_pose.target_pose.position.y = camera_y
                Target_pose.target_pose.position.z = camera_z + 0.05
                # else:
                #     rot = Rotation.from_euler('xyz', [0, 80, 0], degrees=True)
                #     if Arm == 'right_arm':
                #         Target_pose.target_pose.position.x = camera_x
                #         Target_pose.target_pose.position.y = camera_y + 0.027
                #         Target_pose.target_pose.position.z = camera_z - 0.01
                #     else:
                #         Target_pose.target_pose.position.x = camera_x
                #         Target_pose.target_pose.position.y = camera_y - 0.04
                #         Target_pose.target_pose.position.z = camera_z + 0.05

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


def pose_dis(pose_1, pose_2):
    """Compute distance between pose_1 and pose_2
    """
    x = pose_1[0] - pose_2[0]
    y = pose_1[1] - pose_2[1]
    z = pose_1[2] - pose_2[2]

    dis = math.sqrt(x**2+y**2+z**2)

    return dis

def waypoint(current_pose, Target_pose):
    """Generate a list of way points from current pose to target pose

    Input : current pose, target pose : list [x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori]
    Return : a list of way points

    """
    waypoint_list = []
    factor = 0.5
    sub_pose = copy.deepcopy(current_pose)

    # threshold : distance between sub_pose and target_pose = 0.05 meter
    dis = pose_dis(sub_pose, Target_pose)
    while dis > 0.05:
        sub_pose[0] = (sub_pose[0] + Target_pose[0])*factor
        sub_pose[1] = (sub_pose[1] + Target_pose[1])*factor
        sub_pose[2] = (sub_pose[2] + Target_pose[2])*factor
        sub_pose[3] = Target_pose[3]
        sub_pose[4] = Target_pose[4]
        sub_pose[5] = Target_pose[5]
        sub_pose[6] = Target_pose[6]

        dis = pose_dis(sub_pose, Target_pose)

        waypoint_list.append(copy.deepcopy(sub_pose))

    waypoint_list.append(Target_pose)

    return waypoint_list

def get_line_len(center, depth):
    """Generate grasping two endpoints of grasping line(8cm).

    Inputs:
      theta: degree
      center: x, y coordinate
      depth: depth image (mm)
    """
    depth = depth/1000.0
    if depth[center[0], center[1]] < 0.1:
        dis_ = (np.max(depth) + np.min(depth))*0.5
        dis = dis_ - 0.199
    else:
        dis = depth[center[0], center[1]] - 0.199

    length = int((148 - int(dis*50.955))/2)

    return length

def width_detect(depth, center, theta):
    if np.max(depth) < 10:
        depth = depth*1000
    # print(depth.shape, np.max(depth))
    dis_center = int(depth[center[0], center[1]])
    if dis_center == 0.0:
        dis_center = 313
    depth_line = [dis_center]

    is_width_r = True
    is_width_l = True

    height = 480
    width = 640

    lan = 0

    thes_depth = 100

    i = 0

    LAN = get_line_len(center, depth)
    LAN = LAN*1.8

    while is_width_l == True or is_width_r == True:
        if theta == 0:
            r_pos = center[1] + i
            l_pos = center[1] - i

            c_r = (r_pos, center[0])
            c_l = (l_pos, center[0])

            if abs(r_pos) < width:
                dis_r = int(depth[center[0], r_pos])
            else:
                is_width_r = False
            if abs(l_pos) < width:
                dis_l = int(depth[center[0], l_pos])
            else:
                is_width_l = False

        elif theta == 90:
            r_pos = center[0] + i
            l_pos = center[0] - i

            if abs(r_pos) < height:
                dis_r = int(depth[r_pos, center[1]])
            else:
                is_width_r = False
            if abs(l_pos) < height:
                dis_l = int(depth[l_pos, center[1]])
            else:
                is_width_l = False

            c_r = (center[1], r_pos)
            c_l = (center[1], l_pos)

        elif theta == 45:
            r_pos_x = center[0] + i
            r_pos_y = center[1] - i

            l_pos_x = center[0] - i
            l_pos_y = center[1] + i

            if abs(r_pos_x) < 480 and abs(r_pos_y) < 640:
                dis_r = int(depth[r_pos_x, r_pos_y])
            else:
                is_width_r = False
            if abs(l_pos_x) < 480 and abs(l_pos_y) < 640:
                dis_l = int(depth[l_pos_x, l_pos_y])

            c_r = (r_pos_y, r_pos_x)
            c_l = (l_pos_y, l_pos_x)

        elif theta == -45:
            r_pos_x = center[0] + i
            r_pos_y = center[1] + i

            l_pos_x = center[0] - i
            l_pos_y = center[1] - i

            if abs(r_pos_x) < 480 and abs(r_pos_y) < 640:
                dis_r = int(depth[r_pos_x, r_pos_y])
            else:
                is_width_r = False
            if abs(l_pos_x) < 480 and abs(l_pos_y) < 640:
                dis_l = int(depth[l_pos_x, l_pos_y])
            else:
                is_width_l = False

            c_r = (r_pos_y, r_pos_x)
            c_l = (l_pos_y, l_pos_x)

        depth_line.insert(0, dis_l)
        depth_line.append(dis_r)

        if dis_r != 0:
            if (abs(dis_center-dis_r) < thes_depth) and (is_width_r==True):
                lan += 1
            else:
                is_width_r = False

        if dis_l != 0:
            if (abs(dis_center-dis_l) < thes_depth) and (is_width_l==True):
                lan += 1
            else:
                is_width_l = False


        i += 1

    return lan - LAN