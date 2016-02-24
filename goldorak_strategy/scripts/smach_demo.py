#!/usr/bin/env python


import roslib
import rospy
import argparse

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine
from smach_ros import SimpleActionState, IntrospectionServer

WAYPOINTS = {
    'START': (0., 0., 'END'),
    'END': (0., 2.0, 'START'),
}

def main():
    rospy.init_node('smach_example_state_machine')

    sm = StateMachine(['exit'])
    with sm:
        for key, (x, y, next_point) in WAYPOINTS.items():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.

            StateMachine.add(key, SimpleActionState('move_base',
                                                MoveBaseAction,
                                                goal=goal),
                              transitions={'succeeded': next_point, 'aborted': 'exit', 'preempted': 'exit'})

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
