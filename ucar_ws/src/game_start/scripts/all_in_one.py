#!/usr/bin/env python

from actionlib import simple_action_client
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rospy import client
from rospy.topics import Poller
from smach import StateMachine
from smach_ros import SimpleActionState

import patrol_B
import patrol_D1

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['navigating', 'wait', 'end'])
    
    def execute(self, userdata):
        global g_targets, g_start
        if rospy.is_shutdown():
            return 'end'

        if g_start == True:
            if len(g_targets) > 0:
                return 'navigating'
            else:
                print('No targets, can not start.')
                return 'wait'
        else:
            return 'wait'
