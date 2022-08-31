#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from vx300s_bringup.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, Int16
from scipy.spatial.transform import Rotation as R
import math
from arm_control.srv import *

import tf
import tf.transformations as tfm

move_time = 2.5
gripper_pressure = 0.9
move_step = 0.15

class vx300s_right():
    def __init__(self, name):

        self.name = name

        # Service
        rospy.Service("/{0}/go_home".format(name), Trigger, self.vx300s_home)
        rospy.Service("/{0}/go_sleep".format(name), Trigger, self.vx300s_sleep)
        rospy.Service("/{0}/go_pose".format(name), ee_pose, self.vx300s_ee_pose)
        rospy.Service("/{0}/gripper_open".format(name), Trigger, self.vx300s_open)
        rospy.Service("/{0}/gripper_close".format(name), Trigger, self.vx300s_close)
        rospy.Service("/{0}/check_grasped".format(name), Trigger, self.vx300s_check)
        rospy.Service("/{0}/go_rise".format(name), Trigger, self.vx300s_rise)
        rospy.Service("/{0}/go_handover".format(name), Trigger, self.vx300s_handover)
        rospy.Service("/{0}/go_place".format(name), Trigger, self.vx300s_place)
        rospy.Service("/{0}/camera_pose".format(name), Trigger, self.vx300s_camerahold)
        rospy.Service("/{0}/meetmidcam".format(name), Trigger, self.vx300s_meetmidcam)
        rospy.Service("/{0}/turnto0".format(name), Trigger, self.turn20)
        rospy.Service("/{0}/turnto90".format(name), Trigger, self.turn290)
        rospy.Service("/{0}/turnto45".format(name), Trigger, self.turn245)
        rospy.Service("/{0}/turnto_45".format(name), Trigger, self.turn2_45)
        rospy.Service("/{0}/multi_view_90".format(name), Trigger, self.multi_view_90)
        rospy.Service("/{0}/multi_view_0".format(name), Trigger, self.multi_view_0)
        rospy.Service("/{0}/multi_view_22".format(name), Trigger, self.multi_view_22)
        rospy.Service("/{0}/multi_view_67".format(name), Trigger, self.multi_view_67)
        rospy.Service("/{0}/view3".format(name), Trigger, self.view3)
        rospy.Service("/{0}/forward".format(name), Trigger, self.forward)
        rospy.Service("/{0}/turn".format(name), float_command, self.turn)
        rospy.Service("/{0}/vr_handover".format(name), Trigger, self.vr_handover)

        # Subscriber
        rospy.Subscriber('/joint_states_right', JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber('/vr/right/primarybutton', Bool, self.primarybutton_callback, queue_size=1)
        rospy.Subscriber('/vr/right/secondarybutton', Bool, self.secondarybutton_callback, queue_size=1)
        rospy.Subscriber('/vr/right/forward', Int32, self.forward_callback, queue_size=1)
        rospy.Subscriber('/vr/right/updown', Int32, self.updown_callback, queue_size=1)
        rospy.Subscriber('/vr/left/rotation', Int32, self.rotation_callback, queue_size=1)
        rospy.Subscriber('/vr/ray_view', Int32, self.ray_view_callback, queue_size=1)
        rospy.Subscriber('/vr/start_bottleopen', Int32, self.bottleopen_callback, queue_size=1)

        # Publisher
        self.led_state_pub = rospy.Publisher('/handover_server/led_state', Int16, queue_size=1)

        self.led_state_msg = Int16()

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=name, moving_time=move_time, accel_time=0.1, gripper_pressure=gripper_pressure, init_node=False)

        self.arm = robot.arm
        self.gripper = robot.gripper

        self.now_status = 'init'
        self.bottleopen_status = 'init'
        self.rotation_position = 0

        self.init()

    def turn(self, req):
        res = float_commandResponse()

        # print('Turn : ',req.view.data)

        self.arm.set_single_joint_position('wrist_rotate', req.view.data)

        res.result = 'success'

        return res

    def turn20(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', 0)

        res.success = True

        return res

    def turn290(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', -1.57)

        res.success = True

        return res

    def turn245(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', -0.78)

        res.success = True

        return res

    def turn2_45(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', 0.78)

        res.success = True

        return res

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        rospy.loginfo("initial already!")

    def joint_states_callback(self, msg):
        self.arm.set_trajectory_time(moving_time = 0.7, accel_time = 0.1)

        joint_positions = msg.position[:6:]
        self.arm.set_joint_positions(joint_positions)
        rospy.loginfo("VR control!")

        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        self.arm.set_trajectory_time(moving_time = 2.5, accel_time = 0.1)

    def primarybutton_callback(self, msg):
        if msg.data == True:
            self.gripper.open(2.0)
            rospy.loginfo("gripper open!")
        self.rotation_position = 0

        self.led_state_msg.data = 0
        self.led_state_pub.publish(self.led_state_msg.data)

    def secondarybutton_callback(self, msg):
        if msg.data == True:
            self.gripper.close(2.0)
            rospy.loginfo("gripper close!")
        self.rotation_position = 0

    def forward(self, req):
        rotation_rpy = R.from_matrix(self.arm.get_ee_pose_command()[:3,:3])
        rotation_rpy = rotation_rpy.as_rotvec()

        if(self.now_status == 'view_22'):
            move_step = 0.15
        elif(self.now_status == 'view_0'):
            move_step = 0.11
        else:
            move_step = 0.12

        step_x = move_step * math.cos(rotation_rpy[2])
        step_y = move_step * math.sin(rotation_rpy[2])

        x = self.arm.get_ee_pose_command()[0,3] + step_x
        y = self.arm.get_ee_pose_command()[1,3] + step_y
        z = self.arm.get_ee_pose_command()[2,3]
        self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=rotation_rpy[0], pitch=rotation_rpy[1], yaw=rotation_rpy[2])

        res = TriggerResponse()
        res.success = True

        return res
    def forward_callback(self, msg):
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        rotation_rpy = R.from_matrix(self.arm.get_ee_pose_command()[:3,:3])
        rotation_rpy = rotation_rpy.as_rotvec()

        if(self.now_status == 'view_22'):
            move_step = 0.14
        elif(self.now_status == 'view_0'):
            move_step = 0.11
        else:
            move_step = 0.12

        step_x = move_step * math.cos(rotation_rpy[2])
        step_y = move_step * math.sin(rotation_rpy[2])

        if(msg.data == 1):
            x = self.arm.get_ee_pose_command()[0,3] + step_x
            y = self.arm.get_ee_pose_command()[1,3] + step_y
            z = self.arm.get_ee_pose_command()[2,3]
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=rotation_rpy[0], pitch=rotation_rpy[1], yaw=rotation_rpy[2])
        elif(msg.data == -1):
            x = self.arm.get_ee_pose_command()[0,3] - step_x
            y = self.arm.get_ee_pose_command()[1,3] - step_y
            z = self.arm.get_ee_pose_command()[2,3]
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=rotation_rpy[0], pitch=rotation_rpy[1], yaw=rotation_rpy[2])

    def updown_callback(self, msg):
        rotation_rpy = R.from_matrix(self.arm.get_ee_pose_command()[:3,:3])
        rotation_rpy = rotation_rpy.as_rotvec()

        if(msg.data == 1):
            x = self.arm.get_ee_pose_command()[0,3]
            y = self.arm.get_ee_pose_command()[1,3]
            z = self.arm.get_ee_pose_command()[2,3] + 0.08
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=rotation_rpy[0], pitch=rotation_rpy[1], yaw=rotation_rpy[2])
        elif(msg.data == -1):
            x = self.arm.get_ee_pose_command()[0,3]
            y = self.arm.get_ee_pose_command()[1,3]
            z = self.arm.get_ee_pose_command()[2,3] - 0.08
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=rotation_rpy[0], pitch=rotation_rpy[1], yaw=rotation_rpy[2])
        
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)
    
    def rotation_callback(self, msg):
        if(msg.data == 1):
            self.rotation_position += 0.26
            self.arm.set_single_joint_position('wrist_rotate', self.rotation_position)
        elif(msg.data == -1):
            self.rotation_position -= 0.26
            self.arm.set_single_joint_position('wrist_rotate', self.rotation_position)

    def ray_view_callback(self, msg):
        if(msg.data == 0):
            self.now_status = 'view_0'
            self.arm.set_joint_positions([-0.28378644585609436, 0.33900976181030273, 0.04448544234037399, -1.5217089653015137, -1.6198837757110596, 1.1382137537002563])
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)
        elif(msg.data == 1):
            self.now_status = 'view_22'
            self.arm.set_joint_positions([-0.6366020441055298, 0.21629129350185394, 1.0707186460494995, -1.5201749801635742, -1.7011847496032715, 0.25617480278015137])
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)
        elif(msg.data == 2):
            self.now_status = 'view_45'
            self.arm.set_joint_positions([0.7838642, -1.583068, 1.5631264448165894, 0.09050486981868744, -0.1288543939590454, -0.08283496648073196])
            self.led_state_msg.data = 0
            self.led_state_pub.publish(self.led_state_msg.data)
        elif(msg.data == 3):
            self.now_status = 'view_67'
            self.arm.set_joint_positions([1.4818254, 0.122718, 1.2333205938339233, 0.9602720141410828, -1.6843109130859375, -0.14726215600967407])
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)
        elif(msg.data == 4):
            self.now_status = 'view_90'
            self.arm.set_joint_positions([1.7579420, 0.250038, 0.4801360070705414, 1.5569905042648315, -1.6091458797454834, -0.7838642001152039])
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)

    def bottleopen_callback(self, msg):
        self.arm.set_trajectory_time(moving_time = 1.5, accel_time = 0.1)

        if(msg.data == 0 and self.bottleopen_status == 'init'):
            self.bottleopen_status = 'home'
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)
            self.arm.set_ee_pose_components(x=0.25, y=0.0, z=0.2, roll=0.0, pitch=0.0, yaw=0.0)
            self.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.3, roll=0.0, pitch=0.524, yaw=0.0)
        elif(msg.data == 1):
            self.bottleopen_status = 'open'
            self.arm.set_single_joint_position('wrist_rotate', 1.05)
            self.gripper.close(1.0)
            self.arm.set_single_joint_position('wrist_rotate', -1.05)
            self.gripper.open(1.0)
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)
        elif(msg.data == 2 and self.bottleopen_status == 'home'):
            self.bottleopen_status = 'init'
            self.arm.set_joint_positions([0.7838642, -1.583068, 1.5631264448165894, 0.09050486981868744, -0.1288543939590454, -0.08283496648073196])
            self.arm.set_ee_pose_components(x=0.0, y=0.3, z=0.3, roll=0.0, pitch=0.524, yaw=1.571)
            self.led_state_msg.data = 1
            self.led_state_pub.publish(self.led_state_msg.data)

        self.arm.set_trajectory_time(moving_time = 2.5, accel_time = 0.1)
            
    def vx300s_rise(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([-0.04908738657832146, -0.5660389065742493, 0.5460971593856812, 0.05522330850362778, -0.21629129350185394, -0.012271846644580364])
        res.success = True
        
        return res

    def vx300s_place(self, req):
        res = TriggerResponse()

        # self.arm.set_joint_positions([0.8528933525085449, -0.03528155758976936, 0.653475821018219, -0.05368933081626892, -0.6427379846572876, 0.010737866163253784])

        # rospy.sleep(0.5)

        self.arm.set_joint_positions([0.003067961661145091, -0.1426602154970169, 1.1612235307693481, 0.00920388475060463, -0.849825382232666, 0.09970875084400177])
        res.success = True
        
        return res

    def vx300s_handover(self, req):
        self.now_status = 'view_45'
        res = TriggerResponse()
        
        self.arm.set_joint_positions([0.7838642001152039, -1.5830682516098022, 1.5631264448165894, 0.09050486981868744, -0.1288543939590454, -0.08283496648073196])
        res.success = True

        # self.led_state_msg.data = 0
        # self.led_state_pub.publish(self.led_state_msg.data)

        return res

    def multi_view_90(self, req):
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        self.now_status = 'view_90'
        res = TriggerResponse()
        
        self.arm.set_joint_positions([1.7579420804977417, 0.25003886222839355, 0.4801360070705414, 1.5569905042648315, -1.6091458797454834, -0.7838642001152039])
        # self.arm.set_joint_positions([1.7594760656356812, 0.29452431201934814, 0.4770680367946625, 1.5355148315429688, -1.5922720432281494, -0.8360195755958557])
        
        res.success = True

        return res
        
    def multi_view_67(self, req):
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        self.now_status = 'view_67'
        res = TriggerResponse()
        
        # self.arm.set_joint_positions([0.0, -0.8007379770278931, 0.9249904155731201, 0.0015339808305725455, -0.06596117466688156, 0.0015339808305725455])
        self.arm.set_joint_positions([1.4818254709243774, 0.12271846830844879, 1.2333205938339233, 0.9602720141410828, -1.6843109130859375, -0.14726215600967407])
        
        res.success = True

        return res

    def multi_view_22(self, req):
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        self.now_status = 'view_22'
        res = TriggerResponse()
        # -0.6135923266410828, 0.28685441613197327, 0.8789710402488708, -1.6873788833618164, -1.6843109130859375, 0.39269909262657166
        
        self.arm.set_joint_positions([-0.6366020441055298, 0.21629129350185394, 1.0707186460494995, -1.5201749801635742, -1.7011847496032715, 0.25617480278015137])
        # self.arm.set_joint_positions([-0.49854376912117004, -0.0076699042692780495, 0.9081166386604309, -1.3499031066894531, -1.6014759540557861, 0.6458059549331665])
        
        res.success = True

        return res

    def multi_view_0(self, req):
        self.led_state_msg.data = 1
        self.led_state_pub.publish(self.led_state_msg.data)

        self.now_status = 'view_0'
        res = TriggerResponse()
        # -0.3206019997596741, 0.5292233824729919, -0.2040194571018219, -1.451145887374878, -1.667437195777893, 1.2379225492477417
        # -0.4279806613922119, 0.5721748471260071, -0.3850291967391968, 1.5692623853683472, 1.872990608215332, -1.745670199394226
        # -0.3911651074886322, 0.6902913451194763, -0.46019425988197327, 1.6030099391937256, 1.954291582107544, -1.7794177532196045
        self.arm.set_joint_positions([-0.28378644585609436, 0.33900976181030273, 0.04448544234037399, -1.5217089653015137, -1.6198837757110596, 1.1382137537002563])
        # self.arm.set_joint_positions([-0.2929903268814087, 0.37582531571388245, 0.08590292930603027, -1.6106798648834229, -1.6122138500213623, 1.1274758577346802])
        
        res.success = True

        return res

    def view3(self, req):
        res = TriggerResponse()
        
        # self.arm.set_joint_positions([-1.0385050773620605, 0.39576706290245056, -0.26384469866752625, 1.7502721548080444, 1.8683886528015137, -1.702718734741211])
        # self.arm.set_joint_positions([-0.6089903712272644, -0.5951845645904541, 0.7363107800483704, 1.7840197086334229, 0.9602720141410828, -1.7763497829437256])
        self.arm.set_joint_positions([-1.0185632705688477, 0.14112623035907745, 0.7332428097724915, 1.8116313219070435, 1.6106798648834229, -2.4651072025299072])
        res.success = True

        return res

    def vr_handover(self, req):
        res = TriggerResponse()
        
        self.arm.set_joint_positions([-0.016873789951205254, -0.6197282671928406, 0.7869321703910828, -0.13192234933376312, 0.5905826091766357, 0.06596117466688156])
        res.success = True

        return res

    def vx300s_camerahold(self, req):

        res = TriggerResponse()
        self.arm.set_ee_pose_components(x=0.2, y=0, z=0.4, roll=0, pitch=1, yaw=0)
        res.success = True

        return res

    def vx300s_meetmidcam(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([0.3, 0.8, -0.2, -0.5, -1.7, 0])
        res.success = True

        return res

    def vx300s_home(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_home_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_sleep(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_sleep_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)

        self.led_state_msg.data = 0
        self.led_state_pub.publish(self.led_state_msg.data)
        
        return res

    def vx300s_check(self, req):
        
        res = TriggerResponse()

        try:
            joint_info = rospy.wait_for_message('/{0}/joint_states'.format(self.name), JointState)
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)

        if(joint_info.position[6] <= 1.39 and joint_info.position[6] >= -0.42):
            res.success = True
            rospy.loginfo("grasped object")
        else:
            res.success = False
            rospy.loginfo("no object grasped")

        return res

    def vx300s_open(self, req):

        res = TriggerResponse()

        try:
            self.gripper.open(0.5)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_close(self, req):

        res = TriggerResponse()

        try:
            self.gripper.close(0.5)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_ee_pose(self, req):

        res = ee_poseResponse()

        try:
            x = req.target_pose.position.x
            y = req.target_pose.position.y
            z = req.target_pose.position.z
            ox = req.target_pose.orientation.x
            oy = req.target_pose.orientation.y
            oz = req.target_pose.orientation.z
            ow = req.target_pose.orientation.w
            roll, pitch, yaw = tfm.euler_from_quaternion([ox, oy, oz, ow])
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
            res.result = "success"
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.result = "Fail"
            print("Service call failed: %s"%e)
        
        return res

class vx300s_left():
    def __init__(self, name):

        self.name = name

        # Service
        rospy.Service("/{0}/go_home".format(name), Trigger, self.vx300s_home)
        rospy.Service("/{0}/go_sleep".format(name), Trigger, self.vx300s_sleep)
        rospy.Service("/{0}/go_pose".format(name), ee_pose, self.vx300s_ee_pose)
        rospy.Service("/{0}/gripper_open".format(name), Trigger, self.vx300s_open)
        rospy.Service("/{0}/gripper_close".format(name), Trigger, self.vx300s_close)
        rospy.Service("/{0}/check_grasped".format(name), Trigger, self.vx300s_check)
        rospy.Service("/{0}/go_rise".format(name), Trigger, self.vx300s_rise)
        rospy.Service("/{0}/go_handover".format(name), Trigger, self.vx300s_handover)
        rospy.Service("/{0}/go_place".format(name), Trigger, self.vx300s_place)
        rospy.Service("/{0}/camera_pose".format(name), Trigger, self.vx300s_camerahold)
        rospy.Service("/{0}/meetmidcam".format(name), Trigger, self.vx300s_meetmidcam)
        rospy.Service("/{0}/turnto0".format(name), Trigger, self.turn20)
        rospy.Service("/{0}/turnto90".format(name), Trigger, self.turn290)
        rospy.Service("/{0}/multi_view".format(name), Trigger, self.multi_view)
        rospy.Service("/{0}/view3".format(name), Trigger, self.view3)


        # Subscriber
        rospy.Subscriber('/joint_states_left', JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber('/vr/left/primarybutton', Bool, self.primarybutton_callback, queue_size=1)
        rospy.Subscriber('/vr/left/secondarybutton', Bool, self.secondarybutton_callback, queue_size=1)

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=name, moving_time=move_time, accel_time=0.1, gripper_pressure=gripper_pressure, init_node=False)

        self.arm = robot.arm
        self.gripper = robot.gripper

        self.init()

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        rospy.loginfo("initial already!")


    def turn20(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', 0)

        res.success = True

        return res

    def view3(self, req):
        res = TriggerResponse()
        
        # self.arm.set_joint_positions([-1.0385050773620605, 0.39576706290245056, -0.26384469866752625, 1.7502721548080444, 1.8683886528015137, -1.702718734741211])
        self.arm.set_joint_positions([1.167359471321106, 0.4049709439277649, 0.3374757766723633, -1.5355148315429688, 1.9650294780731201, 2.2580196857452393])
        res.success = True

        return res

    def turn290(self, req):
        res = TriggerResponse()

        self.arm.set_single_joint_position('wrist_rotate', -1.57)

        res.success = True

        return res


    def multi_view(self, req):
        res = TriggerResponse()
        
        # self.arm.set_joint_positions([-0.05522330850362778, -1.3314954042434692, 0.7240389585494995, 0.09970875084400177, 0.676485538482666, -0.09050486981868744])
        self.arm.set_joint_positions([-0.003067961661145091, -1.8469128608703613, 1.4557478427886963, -0.05675728991627693, 0.23316508531570435, 0.03988350182771683])

        res.success = True

        return res

    

    def joint_states_callback(self, msg):

        joint_positions = msg.position[:6:]
        self.arm.set_joint_positions(joint_positions)
        rospy.loginfo("VR control!")

    def primarybutton_callback(self, msg):

        if msg.data == True:
            self.gripper.open(2.0)
            rospy.loginfo("gripper open!")

    def secondarybutton_callback(self, msg):

        if msg.data == True:
            self.gripper.close(2.0)
            rospy.loginfo("gripper close!")

    def vx300s_home(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_home_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_rise(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([-0.029145635664463043, -0.1871456652879715, 0.11351457983255386, -0.13345633447170258, -0.07516506314277649, 0.12271846830844879])
        res.success = True
        
        return res

    def vx300s_place(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([0.6028544902801514, 1.0400390625, -0.7010292410850525, 1.1566215753555298, -0.6350680589675903, -1.055378794670105])
        res.success = True
        
        return res

    def vx300s_handover(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([-0.5276893973350525, -0.2040194571018219, 1.2839419841766357, 0.003067961661145091, -1.228718638420105, 0.0015339808305725455])
        res.success = True

        return res

    def vx300s_camerahold(self, req):

        res = TriggerResponse()
        self.arm.set_ee_pose_components(x=0.2, y=0, z=0.4, roll=0, pitch=1, yaw=0)
        res.success = True

        return res


    def vx300s_sleep(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_sleep_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_meetmidcam(self, req):
        res = TriggerResponse()

        self.arm.set_joint_positions([-0.4, 0.2, 0.5, -0.5, -1.5, 0])
        
        res.success = True

        return res

    def vx300s_check(self, req):
        
        res = TriggerResponse()

        try:
            joint_info = rospy.wait_for_message('/{0}/joint_states'.format(self.name), JointState)
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)

        if(joint_info.position[6] <= 1.39 and joint_info.position[6] >= -0.42):
            res.success = True
            rospy.loginfo("grasped object")
        else:
            res.success = False
            rospy.loginfo("no object grasped")

        return res

    def vx300s_open(self, req):

        res = TriggerResponse()

        try:
            self.gripper.open(0.5)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_close(self, req):

        res = TriggerResponse()

        try:
            self.gripper.close(0.5)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_ee_pose(self, req):

        res = ee_poseResponse()

        try:
            x = req.target_pose.position.x
            y = req.target_pose.position.y
            z = req.target_pose.position.z
            ox = req.target_pose.orientation.x
            oy = req.target_pose.orientation.y
            oz = req.target_pose.orientation.z
            ow = req.target_pose.orientation.w
            roll, pitch, yaw = tfm.euler_from_quaternion([ox, oy, oz, ow])
            self.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
            res.result = "success"
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.result = "Fail"
            print("Service call failed: %s"%e)
        
        return res

if __name__=='__main__':

    rospy.init_node("dualarm_control_node", anonymous=False)

    robot_name = rospy.get_param("right")
    VX300s = vx300s_right(robot_name)

    # robot_name = rospy.get_param("left")
    # VX300s = vx300s_left(robot_name)
    
    rospy.spin()
