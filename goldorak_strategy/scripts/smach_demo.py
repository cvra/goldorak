#!/usr/bin/env python

import time
import argparse
from math import radians

import rospy
import roslib
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine, State

from smach_ros import SimpleActionState, IntrospectionServer

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

class Transitions:
    SUCCESS = 'succeeded'
    FAILURE = 'failure'

class WaitStartState(State):
    def __init__(self):
        State.__init__(self, outcomes=[Transitions.SUCCESS])

    def execute(self, userdata):
        rospy.loginfo('Waiting for start')
        rospy.loginfo('starting')
        return Transitions.SUCCESS

def add_waypoints(waypoints):
    for key, (x, y, next_point) in waypoints:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(90)))

        StateMachine.add(key, SimpleActionState('move_base',
                         MoveBaseAction,
                         goal=goal),
                         transitions={
                             Transitions.SUCCESS: next_point,
                             'aborted': Transitions.FAILURE,
                             'preempted': Transitions.FAILURE}
                         )


def create_door_state_machine(door_x):
    sm = StateMachine(outcomes=[Transitions.SUCCESS, Transitions.FAILURE])

    waypoints = (
        ('approach', (door_x, 1.5, 'close')),
        ('close', (door_x, 1.8, 'back_out')),
        ('back_out', (door_x, 1.5, Transitions.SUCCESS)),
    )

    with sm:
        add_waypoints(waypoints)

    return sm

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
        StateMachine.add('waiting', WaitStartState(), transitions={Transitions.SUCCESS: 'inner_door'})

        StateMachine.add('inner_door', create_door_state_machine(0.3),
                transitions={Transitions.FAILURE: 'exit',
                             Transitions.SUCCESS: 'outer_door'})

        StateMachine.add('outer_door', create_door_state_machine(0.6),
                transitions={Transitions.FAILURE: 'exit',
                             Transitions.SUCCESS: 'exit'})

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sm, '/strat')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
