#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *
from handover.srv import *



def multi_view_grasp():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=0)),
                            {'succeeded':'Multi-view Detect','aborted':'Init'})

        smach.StateMachine.add('Multi-view Detect',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=1)),
                            {'succeeded':'Go target','aborted':'Multi-view Detect'})

        smach.StateMachine.add('Go target',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=2)),
                            {'succeeded':'Grasp and back','aborted':'forward'})

        smach.StateMachine.add('forward',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=7)),
                            {'succeeded':'wait object','aborted':'wait object'})

        smach.StateMachine.add('wait object',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=5)),
                            {'succeeded':'Grasp and back','aborted':'wait object'})


        smach.StateMachine.add('Grasp and back',
                            smach_ros.SimpleActionState('handover_action', TestAction,
                            goal = TestGoal(goal=3)),
                            {'succeeded':'End','aborted':'End'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/Multi_view')
    sis.start()

    outcome = sm0.execute()
    sis.stop()

    
if __name__ == '__main__':
    multi_view_grasp()


