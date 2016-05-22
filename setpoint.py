#!/usr/bin/env python

import time
import rospy
from cvra_msgs.msg import MotorControlSetpoint

def talker():
    rospy.init_node('setpoint_sender')

    # Initialise robot pose
    pub = rospy.Publisher('/fishing_y_axis/setpoint', MotorControlSetpoint, queue_size=1)
    time.sleep(1)

    setpoint = 2
    counter = 0

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = MotorControlSetpoint()
        msg.timestamp = rospy.get_rostime()
        msg.node_name = 'fishing_y_axis'
        msg.mode = MotorControlSetpoint.MODE_CONTROL_POSITION
        msg.position = setpoint + 8
        rospy.loginfo('Sending setpoint')
        pub.publish(msg)
        rate.sleep()
        counter += 1

        if counter == 5:
            setpoint = - setpoint
            counter = 0


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
