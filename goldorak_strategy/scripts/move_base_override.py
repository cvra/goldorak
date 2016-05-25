#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from cvra_msgs.msg import MotorControlSetpoint

def init():
    global right_wheel_pub, left_wheel_pub, right_wheel_pos_sub, left_wheel_pos_sub, right_wheel_radius, left_wheel_radius, right_wheel_direction, left_wheel_direction

    right_wheel_pub = rospy.Publisher('/right_wheel/setpoint', MotorControlSetpoint, queue_size=1)
    right_wheel_pos_sub = rospy.Subscriber('/right_wheel/feedback/position', Float32, _right_wheel_pos_cb)

    left_wheel_pub = rospy.Publisher('/left_wheel/setpoint', MotorControlSetpoint, queue_size=1)
    left_wheel_pos_sub = rospy.Subscriber('/left_wheel/feedback/position', Float32, _left_wheel_pos_cb)

    right_wheel_radius = rospy.get_param('diffbase/right_wheel/radius')
    left_wheel_radius = rospy.get_param('diffbase/left_wheel/radius')

    right_wheel_direction = rospy.get_param('diffbase/right_wheel/direction')
    left_wheel_direction = rospy.get_param('diffbase/left_wheel/direction')

    rospy.sleep(1)

def _right_wheel_pos_cb(data):
    global right_wheel_pos
    right_wheel_pos = data.data

def _left_wheel_pos_cb(data):
    global left_wheel_pos
    left_wheel_pos = data.data

def move(x):
    global right_wheel_pub, left_wheel_pub, right_wheel_radius, left_wheel_radius, right_wheel_direction, left_wheel_direction

    right_wheel_msg = MotorControlSetpoint()
    left_wheel_msg = MotorControlSetpoint()

    right_wheel_msg.timestamp = rospy.get_rostime()
    right_wheel_msg.node_name = 'right_wheel'
    right_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION
    right_wheel_msg.position = right_wheel_pos + right_wheel_direction * x / right_wheel_radius

    left_wheel_msg.timestamp = rospy.get_rostime()
    left_wheel_msg.node_name = 'left_wheel'
    left_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION
    left_wheel_msg.position = left_wheel_pos + left_wheel_direction * x / left_wheel_radius

    right_wheel_pub.publish(right_wheel_msg)
    left_wheel_pub.publish(left_wheel_msg)
