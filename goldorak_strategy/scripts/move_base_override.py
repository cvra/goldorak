#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from cvra_msgs.msg import MotorControlSetpoint

def init():
    global right_wheel_pub, left_wheel_pub, right_wheel_pos_sub, left_wheel_pos_sub, right_wheel_radius, left_wheel_radius, right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction

    right_wheel_pub = rospy.Publisher('/right_wheel/setpoint', MotorControlSetpoint, queue_size=1)
    right_wheel_pos_sub = rospy.Subscriber('/right_wheel/feedback/position', Float32, _right_wheel_pos_cb)

    left_wheel_pub = rospy.Publisher('/left_wheel/setpoint', MotorControlSetpoint, queue_size=1)
    left_wheel_pos_sub = rospy.Subscriber('/left_wheel/feedback/position', Float32, _left_wheel_pos_cb)

    right_wheel_radius = rospy.get_param('diffbase/right_wheel/radius')
    left_wheel_radius = rospy.get_param('diffbase/left_wheel/radius')

    right_wheel_direction = rospy.get_param('diffbase/right_wheel/direction')
    left_wheel_direction = rospy.get_param('diffbase/left_wheel/direction')

    external_to_internal_wheelbase_encoder_direction = rospy.get_param('diffbase/external_to_internal_wheelbase_encoder_direction')

    rospy.sleep(1)

def _right_wheel_pos_cb(data):
    global right_wheel_pos
    right_wheel_pos = data.data

def _left_wheel_pos_cb(data):
    global left_wheel_pos
    left_wheel_pos = data.data

def move_speed(speed, duration, rate=10):
    global right_wheel_pub, left_wheel_pub, right_wheel_radius, left_wheel_radius, right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction
    right_speed = external_to_internal_wheelbase_encoder_direction * right_wheel_direction * speed / right_wheel_radius
    left_speed = external_to_internal_wheelbase_encoder_direction * left_wheel_direction * speed / left_wheel_radius

    right_wheel_msg = MotorControlSetpoint()
    left_wheel_msg = MotorControlSetpoint()

    right_wheel_msg.node_name = 'right_wheel'
    right_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_VELOCITY
    left_wheel_msg.node_name = 'left_wheel'
    left_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_VELOCITY

    rate_ = rospy.Rate(rate)

    for i in range(int(duration * rate)):
        current_time = rospy.get_rostime()

        right_wheel_msg.timestamp = current_time
        right_wheel_msg.velocity = right_speed

        left_wheel_msg.timestamp = current_time
        left_wheel_msg.velocity = left_speed

        right_wheel_pub.publish(right_wheel_msg)
        left_wheel_pub.publish(left_wheel_msg)

        rate_.sleep()


def move(x, duration=0.5, rate=10):
    global right_wheel_pos, left_wheel_pos, right_wheel_pub, left_wheel_pub, right_wheel_radius, left_wheel_radius, right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction

    right_wheel_msg = MotorControlSetpoint()
    left_wheel_msg = MotorControlSetpoint()

    right_wheel_msg.node_name = 'right_wheel'
    right_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION
    left_wheel_msg.node_name = 'left_wheel'
    left_wheel_msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION

    right_wheel_msg.position = right_wheel_pos
    left_wheel_msg.position = left_wheel_pos

    right_increment = (external_to_internal_wheelbase_encoder_direction * right_wheel_direction * x / right_wheel_radius) / (duration * rate)
    left_increment = (external_to_internal_wheelbase_encoder_direction * left_wheel_direction * x / left_wheel_radius) / (duration * rate)

    rate_ = rospy.Rate(rate)

    for i in range(int(duration * rate)):
        current_time = rospy.get_rostime()

        right_wheel_msg.timestamp = current_time
        right_wheel_msg.position += right_increment

        left_wheel_msg.timestamp = current_time
        left_wheel_msg.position += left_increment

        right_wheel_pub.publish(right_wheel_msg)
        left_wheel_pub.publish(left_wheel_msg)

        rate_.sleep()
