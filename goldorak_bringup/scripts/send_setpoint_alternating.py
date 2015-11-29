#!/usr/bin/env python
import rospy
from cvra_msgs.msg import MotorControlSetpoint

node_name = 'right_wheel'
frequency = 50

def talker():
    pub = rospy.Publisher('/goldorak/' + node_name + '/setpoint',
                          MotorControlSetpoint,
                          queue_size=10)
    rospy.init_node('setpoint_sender', anonymous=True)
    rate = rospy.Rate(frequency)

    i = 0
    msg = MotorControlSetpoint()
    while not rospy.is_shutdown():
        msg.node_name = node_name
        msg.mode = MotorControlSetpoint.MODE_CONTROL_TORQUE

        if i > frequency:
            msg.position = 0
            msg.velocity = 0
            msg.torque = 20
        if i > 2 * frequency:
            i = 0
            msg.position = 0
            msg.velocity = 0
            msg.torque = -20

        i += 1
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
