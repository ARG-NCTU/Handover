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

def get_pcd_right(rgb_np, depth_np):
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
    rgb = o3d.geometry.Image(rgb_np)
    depth = o3d.geometry.Image(depth_np)

    # rotate_matrix = [[0.8528686, -0.4924039, -0.1736482,0.01],[0.5000000,  0.8660254,  0.0000000,0.0],[0.1503837, -0.0868241,  0.9848077,0.03],[0,0,0,1]]
    # rotate_matrix = [[0.9254166, -0.3368241, -0.1736482,0.2],[0.2655843,  0.9033352, -0.3368241,0.148],[0.2703130,  0.2655843,  0.9254166,0.08],[0,0,0,1]]

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, convert_rgb_to_intensity=False)
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    points1 = np.asarray(pcd1.points)
    pcd_sel1 = pcd1.select_by_index(np.where((points1[:, 2] > 0.25)&(points1[:, 2] < 1.3))[0])

    # pcd_sel1.transform(rotate_matrix)
    # axis.transform(rotate_matrix)

    return pcd_sel1, axis

def get_pcd_left(rgb_np, depth_np):
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
    rgb = o3d.geometry.Image(rgb_np)
    depth = o3d.geometry.Image(depth_np)

    rotate_matrix = [[0.0000000,  0.0000000,  -1.0000000,0.885],[0.0000000,  1.0000000,  0.0000000,0.03],[1.0000000,  0.0000000,  0.0000000,0.665],[0,0,0,1]]

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, convert_rgb_to_intensity=False)
    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    points2 = np.asarray(pcd2.points)
    pcd_sel2 = pcd2.select_by_index(np.where((points2[:, 2] < 2)&(points2[:, 0] < 1.5))[0])
    pcd_sel2.transform(rotate_matrix)

    axis.transform(rotate_matrix)

    return pcd_sel2, axis

def get_view(pcd1, pcd2, view_id):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.Color
    vis.add_geometry(pcd1)
    vis.update_geometry(pcd1)
    vis.add_geometry(pcd2)
    vis.update_geometry(pcd2)
    ctr = vis.get_view_control()
    ctr.set_lookat(view_lookat[view_id])
    ctr.set_front(view_list[view_id])
    ctr.set_up(view_up[view_id])
    ctr.set_zoom(zoom_list[view_id])
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    vis.update_renderer()
    vis.poll_events()
    image = vis.capture_screen_float_buffer(True)
    depth = vis.capture_depth_float_buffer(True)

    img = np.array(image)[414:695,740:1181]
    depth = np.array(depth)[414:695,740:1181]
    img = cv2.resize(img, (640,480))
    depth = cv2.resize(depth, (640,480))

    return img*255, depth

def processing(color, depth):
    color = cv2.resize(color, (224, 224))
    depth = cv2.resize(depth, (224, 224))
    depth[depth > 1000] = 0.0

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


def aff_process(pred, color, depth):
    value = np.max(pred)
    graspable = cv2.resize(pred, (640, 480))
    graspable[depth==0] = 0
    graspable[graspable>=1] = 0.99999
    graspable[graspable<0] = 0
    graspable = cv2.GaussianBlur(graspable, (7, 7), 0)
    affordanceMap = (graspable/np.max(graspable)*255).astype(np.uint8)
    affordanceMap = cv2.applyColorMap(affordanceMap, cv2.COLORMAP_JET)
    affordanceMap = affordanceMap[:,:,[2,1,0]]
    combine = cv2.addWeighted(color,0.7,affordanceMap, 0.3,0,dtype=cv2.CV_8UC3)

    gray = cv2.cvtColor(affordanceMap, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    binaryIMG = cv2.Canny(blurred, 20, 160)
    _, contours, _ = cv2.findContours(binaryIMG, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    i = 0
    point_x = 0
    point_y = 0
    cX = 0
    cY = 0
    x = 0
    y = 0
    
    for c in contours:
        M = cv2.moments(c)
        if(M["m00"]!=0): 
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            zc = depth[cY, cX]/1000
            # if 0 < zc < 0.65:
            i += 1
            point_x += cX
            point_y += cY

    if i != 0:
        x = int(point_x / i)
        y = int(point_y / i)
    else:
        # print(pred.shape)
        x, y = np.where(pred==np.max(pred))
        x = int(x*(640/112))
        y = int(y*(480/112))

    return x, y, combine, value

class Affordance_predict():
    def __init__(self, arm, fx, fy, cx, cy, Mode):
        r = rospkg.RosPack()
        self.path = r.get_path("handover")
        self.net = HANet()
        self.Mode = Mode
        if self.Mode == 'handover':
            self.net.load_state_dict(torch.load(self.path+'/src/HANet/HANet_VALk.pth'))
            self.A = [90,45,0,-45]
            rospy.loginfo('Handover Mode')
        else:
            self.net.load_state_dict(torch.load(self.path+'/src/HANet/HANet_bottel_cap.pth'))
            self.A = [0,0,0,0]
            rospy.loginfo('Open Cover Mode')
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
        # self.sn2q = SurfaceNormal2Quaternion(fx, fy, cx, cy)


    def switch(self):
        if self.go_loop:
            rospy.loginfo("Enable Open Loop")
            self.go_loop = False
        else:
            rospy.loginfo("Enable Close Loop")
            self.go_loop = True

    def switch_hand(self):
        if self.arm == 'right_arm':
            rospy.loginfo("Switch to left arm")
            self.arm = 'left_arm'
        else:
            rospy.loginfo("Switch to right arm")
            self.arm = 'right_arm'
        

    def predict(self, color, depth):
        # Convert msg type
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(color, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth, "16UC1")
            cv_depth_grasp = cv_depth.copy()
            image_pub = cv_image.copy()

        except CvBridgeError as e:
            print(e)
            return

        # Do prediction
        color_in, depth_in = processing(cv_image, cv_depth)
        color_in = color_in.cuda()
        depth_in = depth_in.cuda()

        predict = self.net(color_in, depth_in)

        predict = predict.cpu().detach().numpy()

        Max = []
        for i in range(4):
            Max.append(np.max(predict[0][i]))

        pred_id = Max.index(max(Max))

        # Get gripping point base on camera link
        x, y, aff_pub, self.value = aff_process(predict[0][pred_id], image_pub, cv_depth_grasp)

        if x != 0 and y!=0:
            z = cv_depth_grasp[int(y), int(x)]/1000.0

            aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
            p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")

            camera_x, camera_y, camera_z = self.getXYZ(x, y, z)
            self.target_cam_dis = camera_z


            # rot_quat = self.sn2q.get_quaternion(cv_depth_grasp, x, y, A[pred_id])

            # Add to pose msgs
            Target_pose = ee_poseRequest()

            if self.Mode == 'handover':
                rot = Rotation.from_euler('xyz', [self.A[pred_id], 0, 0], degrees=True)
                Target_pose.target_pose.position.x = camera_x - 0.02
                Target_pose.target_pose.position.y = camera_y - 0.02
                Target_pose.target_pose.position.z = camera_z - 0.05
                # Target_pose.target_pose.position.z = camera_z + 0.1
            else:
                rot = Rotation.from_euler('xyz', [0, 80, 0], degrees=True)
                if self.arm == 'right_arm':
                    # Target_pose.target_pose.position.x = camera_x
                    Target_pose.target_pose.position.x = camera_x - 0.02
                    Target_pose.target_pose.position.y = camera_y + 0.027
                    # Target_pose.target_pose.position.z = camera_z - 0.01
                    Target_pose.target_pose.position.z = camera_z + 0.02
                else:
                    # Target_pose.target_pose.position.x = camera_x
                    Target_pose.target_pose.position.x = camera_x - 0.02
                    Target_pose.target_pose.position.y = camera_y - 0.04
                    Target_pose.target_pose.position.z = camera_z + 0.05

            rot_quat = rot.as_quat()

            Target_pose.target_pose.orientation.x = rot_quat[0]
            Target_pose.target_pose.orientation.y = rot_quat[1]
            Target_pose.target_pose.orientation.z = rot_quat[2]
            Target_pose.target_pose.orientation.w = rot_quat[3]

            target_pose, go_ok = self.camera2world(Target_pose)

            if z == 0.0:
                go_ok = False
            
            return target_pose, go_ok, self.target_cam_dis, p, self.value
        else:
            return None, False, self.target_cam_dis, None, 0

    def predict_mul(self, cv_image, cv_depth, Arm, single=False):
        # Convert msg type
        cv_depth_grasp = cv_depth.copy()
        image_pub = cv_image.copy()

        # Do prediction
        color_in, depth_in = processing(cv_image, cv_depth)
        color_in = color_in.cuda()
        depth_in = depth_in.cuda()

        predict = self.net(color_in, depth_in)

        predict = predict.cpu().detach().numpy()

        Max = []
        for i in range(4):
            Max.append(np.max(predict[0][i]))

        pred_id = Max.index(max(Max))

        # Get gripping point base on camera link
        x, y, aff_pub, self.value = aff_process(predict[0][pred_id], image_pub, cv_depth_grasp)

        if single:
            aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
            p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")
            return p, self.value, self.A[pred_id], [int(y), int(x)]
        else:
            if x != 0 and y!=0:
                z = cv_depth_grasp[int(y), int(x)]/1000.0

                aff_pub = cv2.circle(aff_pub, (int(x), int(y)), 10, (0,255,0), -1)
                p = self.bridge.cv2_to_imgmsg(aff_pub, "bgr8")

                camera_x, camera_y, camera_z = self.getXYZ(x, y, z)
                self.target_cam_dis = camera_z


                # rot_quat = self.sn2q.get_quaternion(cv_depth_grasp, x, y, A[pred_id])

                # Add to pose msgs
                Target_pose = ee_poseRequest()

                if self.Mode == 'handover':
                    rot = Rotation.from_euler('xyz', [self.A[pred_id], -15, 0], degrees=True)
                    # rot = Rotation.from_euler('xyz', [0, -15, 0], degrees=True)
                    # if self.go_loop:
                    #     Target_pose.target_pose.position.x = camera_x - 0.08
                    # else:
                    Target_pose.target_pose.position.x = camera_x - 0.02
                    # Target_pose.target_pose.position.y = camera_y - 0.02
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

                # print('rrrr',Target_pose.target_pose.position.x)

                if z == 0.0 or Target_pose.target_pose.position.x > 3.0:
                    # print('rrrr',Target_pose.target_pose.position.x)
                    go_ok = False
                    return None, False, self.target_cam_dis, None, 0, self.A[pred_id], [int(y), int(x)]
                print('111')
                return target_pose, go_ok, self.target_cam_dis, p, self.value, self.A[pred_id], [int(y), int(x)]
            else:
                print('222')
                return None, False, self.target_cam_dis, None, 0, self.A[pred_id], [int(y), int(x)]

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
            # tf_pose.target_pose.orientation.x = (tf_pose.target_pose.orientation.x + rot_quat[0])*self.factor
            # tf_pose.target_pose.orientation.y = (tf_pose.target_pose.orientation.y + rot_quat[1])*self.factor
            # tf_pose.target_pose.orientation.z = (tf_pose.target_pose.orientation.z + rot_quat[2])*self.factor
            # tf_pose.target_pose.orientation.w = (tf_pose.target_pose.orientation.w + rot_quat[3])*self.factor

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


class HANet(nn.Module):
    class FCN_model(nn.Module):
        def __init__(self, n_classes=4):
            super(HANet.FCN_model, self).__init__()
            self.color_trunk = torchvision.models.resnet101(pretrained=True)
            del self.color_trunk.fc, self.color_trunk.avgpool, self.color_trunk.layer4
            self.depth_trunk = copy.deepcopy(self.color_trunk)
            self.conv1 = nn.Conv2d(2048, 512, 1)
            self.conv2 = nn.Conv2d(512, 128, 1)
            self.conv3 = nn.Conv2d(128, n_classes, 1)
        def forward(self, color, depth):
            # Color
            color_feat_1 = self.color_trunk.conv1(color) # 3 -> 64
            color_feat_1 = self.color_trunk.bn1(color_feat_1)
            color_feat_1 = self.color_trunk.relu(color_feat_1)
            color_feat_1 = self.color_trunk.maxpool(color_feat_1)
            color_feat_2 = self.color_trunk.layer1(color_feat_1) # 64 -> 256
            color_feat_3 = self.color_trunk.layer2(color_feat_2) # 256 -> 512
            color_feat_4 = self.color_trunk.layer3(color_feat_3) # 512 -> 1024
            # Depth
            depth_feat_1 = self.depth_trunk.conv1(depth) # 3 -> 64
            depth_feat_1 = self.depth_trunk.bn1(depth_feat_1)
            depth_feat_1 = self.depth_trunk.relu(depth_feat_1)
            depth_feat_1 = self.depth_trunk.maxpool(depth_feat_1)
            depth_feat_2 = self.depth_trunk.layer1(depth_feat_1) # 64 -> 256
            depth_feat_3 = self.depth_trunk.layer2(depth_feat_2) # 256 -> 512
            depth_feat_4 = self.depth_trunk.layer3(depth_feat_3) # 512 -> 1024
            # Concatenate
            feat = torch.cat([color_feat_4, depth_feat_4], dim=1) # 2048
            feat_1 = self.conv1(feat)
            feat_2 = self.conv2(feat_1)
            feat_3 = self.conv3(feat_2)
            return nn.Upsample(scale_factor=2, mode="bilinear")(feat_3)

    def __init__(self, pretrained=False, n_class=4):
        super(HANet, self).__init__()
        if pretrained == True:
            self.net = self.FCN_model(n_classes=4)
            # model_path = get_model()
            self.net.load_state_dict(torch.load('/home/dualarm/handover-system/model/HANet.pth'))
            print('Load pretrained complete')
        else:
            self.net = self.FCN_model(n_classes=n_class)

    def forward(self, Color, Depth):
        output = self.net(Color, Depth)

        return output