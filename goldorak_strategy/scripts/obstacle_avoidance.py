#!/usr/bin/env python
from __future__ import division
import rospy
import math

from actionlib_msgs.msg import GoalID
from cvra_msgs.msg import BeaconSignal
from geometry_msgs.msg import Twist
from people_msgs.msg import Person, People


def obstacle_within_vision_cone(vel_x, distance, angle, safety_distance, safety_half_angle):
    # Ignore what is outside the danger zone
    if distance < safety_distance:
        if vel_x > 0 and abs(angle % math.radians(360)) < safety_half_angle:
            return True

        if vel_x < 0 and abs((angle % math.radians(360)) - math.radians(180)) < safety_half_angle:
            return True

    return False

def beacon_detection_cb(data):
    global abort_navigation_pub, robots_pub, braking_distance, vision_cone_angle, velocity_msg

    if obstacle_within_vision_cone(velocity_msg.linear.x, data.distance, data.angle,
                                    braking_distance, vision_cone_angle / 2):
        rospy.loginfo('Obstacle detected at [distance:{} angle:{}], closer than {} within vision cone'.format(data.distance, data.angle, braking_distance))

        abort_navigation_pub.publish(GoalID())

    robots_msg = People()
    robots_msg.header.stamp = rospy.get_rostime()
    robots_msg.header.frame_id = "base_link"

    opponent = Person()
    opponent.position.x = data.distance * math.cos(data.angle)
    opponent.position.y = data.distance * math.sin(data.angle)
    robots_msg.people.append(opponent)

    robots_pub.publish(robots_msg)


def velocity_cb(data):
    global velocity_msg
    velocity_msg = data # store velocity, to be used later


def main():
    global abort_navigation_pub, robots_pub, braking_distance, vision_cone_angle, velocity_msg

    velocity_msg = Twist()

    rospy.init_node('obstacle_avoidance_emergency_stop')
    braking_distance = rospy.get_param('obstacle_avoidance/braking_distance', 0.3)
    vision_cone_angle = rospy.get_param('obstacle_avoidance/vision_cone_angle', 1.0)

    rospy.Subscriber("beacon/detection", BeaconSignal, beacon_detection_cb)
    rospy.Subscriber("cmd_vel", Twist, velocity_cb)
    abort_navigation_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

    robots_pub = rospy.Publisher('/people', People, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
