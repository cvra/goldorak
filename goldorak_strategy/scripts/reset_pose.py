#!/usr/bin/env python

from math import radians

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


reset_pose_pub = None

def init():
    global reset_pose_pub

    reset_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1)

def reset(x, y, theta):
    global reset_pose_pub

    if reset_pose_pub == None:
        init()

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta))

    reset_pose_pub.publish(msg)
