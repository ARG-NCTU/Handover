#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import sys

from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *
from handover.srv import *
import argparse

def close_loop_grasp():
    rospy.init_node('handover_client')

    r = TriggerRequest()

    try:
        go_pose = rospy.ServiceProxy("handover_server/switch_loop", Trigger)
        resp = go_pose(r)
    except rospy.ServiceException as exc:
        print("service did not process request: " + str(exc))

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=0)),
                                {'succeeded':'Single detect','aborted':'Init'})

        smach.StateMachine.add('Single detect',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=6)),
                                {'succeeded':'Go target','aborted':'Single detect'})
        
        smach.StateMachine.add('Go target',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=2)),
                                {'succeeded':'Check_dis','aborted':'Single detect'})

        smach.StateMachine.add('Check_dis',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=4)),
                                {'succeeded':'Grasp and back','aborted':'Single detect'})

        smach.StateMachine.add('Grasp and back',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=3)),
                                {'succeeded':'End','aborted':'End'})

    
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/Close-Loop')
    sis.start()

    outcome = sm0.execute()
    sis.stop()

if __name__ == '__main__':
    close_loop_grasp()