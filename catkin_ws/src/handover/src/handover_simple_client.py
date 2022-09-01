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

def simple_grasp():
    rospy.init_node('handover_client')

    r = TriggerRequest()

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=0)),
                                {'succeeded':'Multi-view Detect','aborted':'Init'})

        smach.StateMachine.add('Single detect',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=6)),
                                {'succeeded':'Go target','aborted':'Single detect'})
        
        smach.StateMachine.add('Go target',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=2)),
                                {'succeeded':'Grasp and back','aborted':'Single detect'})

        smach.StateMachine.add('Grasp and back',
                                smach_ros.SimpleActionState('handover_action', TestAction,
                                goal = TestGoal(goal=3)),
                                {'succeeded':'End','aborted':'End'})

    
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()
    sis.stop()

if __name__ == '__main__':
    simple_grasp()