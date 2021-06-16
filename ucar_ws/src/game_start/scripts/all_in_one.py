#!/usr/bin/env python

from actionlib import simple_action_client
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rospy import client
from rospy.topics import Poller
from smach import StateMachine
from smach_ros import SimpleActionState

# class VOICE_AWAKE(SimpleActionState):