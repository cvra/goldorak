#!/usr/bin/env python

import time
import argparse
from math import radians

import rospy
import roslib
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine, State, Sequence

from smach_ros import SimpleActionState, IntrospectionServer

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

class Transitions:
    SUCCESS = 'succeeded'
    FAILURE = 'failure'

class Team:
    GREEN = 'green'
    VIOLET = 'violet'

TEAM = Team.VIOLET

def mirror_point(x, y):
    if TEAM == Team.VIOLET:
        return x, y
    elif TEAM == Team.GREEN:
        return 3.0 - x, y

    raise ValueError("Unknown team")



class WaitStartState(State):
    def __init__(self):
        State.__init__(self, outcomes=[Transitions.SUCCESS])

    def execute(self, userdata):
        rospy.loginfo('Waiting for start')
        rospy.loginfo('starting')
        return Transitions.SUCCESS

def add_waypoints(waypoints):
    for key, (x, y), angle in waypoints:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(angle)))

        Sequence.add(key, SimpleActionState('move_base', MoveBaseAction, goal=goal),
                transitions={'preempted': Transitions.FAILURE, 'aborted': Transitions.FAILURE})


def create_door_state_machine(door_x):
    seq = Sequence(outcomes=[Transitions.SUCCESS, Transitions.FAILURE],
                   connector_outcome=Transitions.SUCCESS)

    waypoints = (
        ('approach', mirror_point(door_x, 1.5), 90),
        ('close', mirror_point(door_x, 1.8), 90),
        ('back_out', mirror_point(door_x, 1.5), 90),
    )

    with seq:
        add_waypoints(waypoints)

    return seq

def main():
    rospy.init_node('smach_example_state_machine')

    # Initialise robot pose
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    time.sleep(1)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    x, y = mirror_point(0.105, 0.900)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

    pub.publish(msg)


    sq = Sequence(outcomes=[Transitions.SUCCESS, Transitions.FAILURE],
                  connector_outcome=Transitions.SUCCESS)
    with sq:
        Sequence.add('waiting', WaitStartState())
        Sequence.add('inner_door', create_door_state_machine(0.3))
        Sequence.add('outer_door', create_door_state_machine(0.6))

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sq, '/strat')
    sis.start()

    # Execute the state machine
    outcome = sq.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
