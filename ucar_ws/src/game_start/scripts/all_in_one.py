#!/usr/bin/env python

from std_msgs.msg import String
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros


def voicecallback(data):
    if data.data == "start":
        print('Start navigating!')
        voiceflag = True
        return voiceflag

    else:
        print('Waiting for voiceAwake......')
        rospy.sleep(3)
        voiceflag = False
        return voiceflag
class Wait4Awake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating', 'wait'])
    
    def execute(self, userdata):
        rospy.Subscriber("voiceAwake",String, voicecallback)
        rospy.spin()
        if voicecallback:
            return 'navigating'
        else:
            return 'wait'




class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating', 'arrived', 'end'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        # global g_start
        if rospy.is_shutdown():
            return 'end'
        else:
            waypoints = self.get_waypoints()
            for pose in waypoints:   
                goal = self.goal_pose(pose)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                rospy.sleep(1.5)
            # g_start = False
            return 'arrived'
    
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
    
    def get_waypoints(self):
        waypoints = [
            [(2.263, -2.945, 0.000),(0.000, 0.000, 0.705, 0.709)]
            ]
        return waypoints


def main():
    # global g_targets, g_start
    # g_targets = []
    # g_start = False
    rospy.init_node('first_try')
    sm = smach.StateMachine(outcomes=['end'])

    with sm:        
        smach.StateMachine.add('WAIT', Wait4Awake(),transitions={'navigating':'NAV', 'wait':'WAIT' })
        smach.StateMachine.add('NAV', Navigate(), transitions={'arrived':'end', 'navigating':'NAV'})

        sis = smach_ros.IntrospectionServer('FIRST_TRY', sm, '/SM_ROOT')
        sis.start()
        
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    # g_targets = None
    # g_start = None
    main()