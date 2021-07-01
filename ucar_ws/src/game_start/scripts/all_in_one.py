#!/usr/bin/env python
#encoding=UTF-8
import cv2
import numpy as np
import cv2.aruco as aruco
import os


from std_msgs.msg import String
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros
import threading

import ht_aruco as ht

voiceflag = False

def thread_job():
    rospy.spin()

def voicecallback(data):
    global voiceflag
    if data.data == "start":
        print('Start navigating!')
        rospy.sleep(1)
        voiceflag = True
        return voiceflag

    else:
        print('Waiting for voiceAwake......')
        rospy.sleep(1)
        voiceflag = False
        return voiceflag


    
class Wait4Awake(smach.State):
    #global voiceflag
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating', 'wait'],output_keys=['navpoints'])
    
    def execute(self, userdata):
        add_thread = threading.Thread(target = thread_job)
        add_thread.start()
        rospy.Subscriber("voiceAwake",String, voicecallback)
        userdata.navpoints = -1
        rospy.sleep(2)

        if voiceflag:
            return 'navigating'
        else:
            return 'wait'




class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating', 'arrived', 'end'],input_keys=['navpoints'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        # global g_start
        if rospy.is_shutdown():
            return 'end'
        else:
        
            waypoints = self.get_waypoints(userdata.navpoints+1) 
            goal = self.goal_pose(waypoints)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.sleep(1.5)
            if (userdata.navpoints+1) == 0:
               return 'arrived'
            elif (userdata.navpoints+1) == 1:
                return 'end'
            elif (userdata.navpoints+1) ==2:
                return 'end'
            elif (userdata.navpoints+1) ==3:
                return 'end'
            
    
    def goal_pose(self, pose): 
        goal_pose = MoveBaseGoal()

        goal_pose.target_pose.header.frame_id = 'map'

        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]

        return goal_pose
    
    def get_waypoints(self,num):
        waypoints = [
            [[2.263, -2.945, 0.000],[0.000, 0.000, 0.705, 0.709]],
            [[1.014, -1.108, 0.000],[0.000, 0.000, 0.732, 0.681]],
            [[0.529, -1.109, 0.000],[0.000, 0.000, 0.749, 0.663]],            
            [[0.005, -1.116, 0.000],[0.000, 0.000, 0.735, 0.678]]
            ]
        return waypoints[num]


class Aruco(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['nav2D1','nav2D2','nav2D3','Aruco','end'],output_keys=['navpoints'])

    def execute(self, userdata):
        count = 0
        count = count+1
        code = ht.ht_aruco()
        if code == 0:
            userdata.navpoints = 0
            return 'nav2D1'
        elif code == 1:
            userdata.navpoints = 1
            return 'nav2D2'
        elif code == 2:
            userdata.navpoints = 2
            return'nav2D3'
        elif code == -1:
            if count <10:
                return 'Aruco'
            else:
                return 'end'



def thread_detect():
    os.system('python3 /mnt/ROS_xunfei/ucar_ws/src/ht_image/scripts/xunfei2.0.py')
class detect(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self, userdata):
        add_thread = threading.Thread(target = thread_detect)
        add_thread.start()
        return 'success'


def main():
    # global g_targets, g_start
    # g_targets = []
    # g_start = False
    rospy.init_node('first_try')
    sm = smach.StateMachine(outcomes=['end'])

    with sm:        
        smach.StateMachine.add('WAIT', Wait4Awake(),transitions={'navigating':'NAV', 'wait':'WAIT' })
        smach.StateMachine.add('NAV', Navigate(), transitions={'arrived':'ARUCO', 'navigating':'NAV','end':'end'})
        smach.StateMachine.add('ARUCO', Aruco(), transitions={'nav2D1':'NAV', 'nav2D2':'NAV','nav2D3':'NAV','Aruco':'ARUCO','end':'end'})


        sis = smach_ros.IntrospectionServer('FIRST_TRY', sm, '/SM_ROOT')
        sis.start()
        
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    # g_targets = None
    # g_start = None
    main()