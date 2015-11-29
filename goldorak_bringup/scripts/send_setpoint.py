#!/usr/bin/env python
"""
Sends a position setpoint using ROS.
"""
import rospy
from cvra_msgs.msg import MotorControlSetpoint
import argparse


node_name = 'right_wheel'
frequency = 100

def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--position", "-p", type=float,
                        help="Position setpoint, in radians.")
    parser.add_argument("--velocity", "-v", type=float,
                        help="Velocity setpoint, in rad/s.")
    parser.add_argument("--torque", "-t", type=float,
                        help="Torque setpoint, in Nm.")

    return parser.parse_args()

def talker():
    args = parse_arguments()

    pub = rospy.Publisher('/goldorak/' + node_name + '/setpoint',
                          MotorControlSetpoint,
                          queue_size=10)
    rospy.init_node('setpoint_sender', anonymous=True)
    rate = rospy.Rate(frequency)

    msg = MotorControlSetpoint()
    while not rospy.is_shutdown():
        msg.node_name = node_name

        if args.position:
            msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION
            msg.position = args.position
        elif args.velocity:
            msg.mode = MotorControlSetpoint.MODE_CONTROL_VELOCITY
            msg.velocity = args.velocity
        elif args.torque:
            msg.mode = MotorControlSetpoint.MODE_CONTROL_TORQUE
            msg.torque = args.torque

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
