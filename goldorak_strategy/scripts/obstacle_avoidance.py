#!/usr/bin/env python
import rospy

from actionlib_msgs.msg import GoalID
from std_msgs.msg import Float32

def callback(data):
    global abort_navigation_pub, braking_distance

    if data.data < braking_distance:
        rospy.loginfo('Obstacle too close, aborting navigation.')
        abort_navigation_pub.publish(GoalID())


def main():
    global abort_navigation_pub, braking_distance

    rospy.init_node('obstacle_avoidance_emergency_stop')
    braking_distance = rospy.get_param('obstacle_avoidance/braking_distance', 0.3)

    rospy.Subscriber("beacon/distance", Float32, callback)
    abort_navigation_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
