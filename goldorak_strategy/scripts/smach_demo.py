#!/usr/bin/env python

import time
import argparse

import rospy
import roslib
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine
from smach_ros import SimpleActionState, IntrospectionServer

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

WAYPOINTS = {
    'START': (0.5, 0.9, 'MIDDLE'),
    'MIDDLE': (0.3, 1.8, 'exit'),
}

def main():
    rospy.init_node('smach_example_state_machine')

    # Initialise robot pose
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    time.sleep(1)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.pose.pose.position.x = 0.105
    msg.pose.pose.position.y = 0.900
    msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

    pub.publish(msg)


    sm = StateMachine(['exit'])
    with sm:
        for key, (x, y, next_point) in WAYPOINTS.items():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
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
