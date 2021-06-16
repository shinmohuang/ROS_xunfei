#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes = ['fuck you!', 'fuck him!'])
        self.counter = 0

def execute(self, userdata):
    rospy.loginfo('Executing Foo')
    if self.counter <3:
        self.counter +=1
        return 'fuck you!'
    else:
        return "fuck him!"

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes = ['fuck you!', 'fuck him!'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing Bar')
        return "fuck him!"

def main():
    rospy.init_node('smach_example_state_machine')

  # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['fuck her!', 'fuck off!'])

  # Open the container
    with sm:
  # Add states to the container
        smach.StateMachine.add('FOO', Foo(), transitions={'fuck you!':'BAR', 'fuck him!':'fuck off!'})
        smach.StateMachine.add('BAR', Bar(), transitions={'fuck him!':'FOO'})

  # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

  # Execute SMACH plan
    outcome = sm.execute()

  # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()