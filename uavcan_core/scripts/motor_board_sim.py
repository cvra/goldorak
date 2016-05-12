#!/usr/bin/env python
from __future__ import division
import sys
import math
import rospy
from std_msgs.msg import Float32
from cvra_msgs.msg import MotorControlSetpoint, MotorEncoderStamped

ENCODER_RESOLUTION = 2**16
REFRESH_RATE = 50
DELTA_T = 1 / REFRESH_RATE
MAX_SPEED = 20.0

class MotorBoardSim:
    def __init__(self, motor_name):
        rospy.init_node('motor_board_sim', anonymous=True)
        self.rate = rospy.Rate(REFRESH_RATE)

        self.vel = 0
        self.pos = 0
        self.enc = 0

        self.vel_pub = rospy.Publisher(motor_name + '/feedback/velocity', Float32, queue_size=1)
        self.pos_pub = rospy.Publisher(motor_name + '/feedback/position', Float32, queue_size=1)
        self.enc_pub = rospy.Publisher(motor_name + '/feedback/encoder', MotorEncoderStamped, queue_size=1)

        rospy.Subscriber(motor_name + '/setpoint', MotorControlSetpoint, self.callback)

    def callback(self, msg):
        if (msg.mode == msg.MODE_CONTROL_VELOCITY):
            self.vel = msg.velocity

        # Clamp velocity
        if self.vel > MAX_SPEED:
            self.vel = MAX_SPEED
        if self.vel < - MAX_SPEED:
            self.vel = - MAX_SPEED

    def update(self):
        self.pos = self.pos + self.vel * DELTA_T
        self.enc = (self.enc + self.vel * DELTA_T * ENCODER_RESOLUTION / math.pi) % ENCODER_RESOLUTION

        self.vel_pub.publish(self.vel)
        self.pos_pub.publish(self.pos)
        self.enc_pub.publish(MotorEncoderStamped(rospy.get_rostime(), self.enc))

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: motor_board_sim.py motor_name")
    else:
        motor = MotorBoardSim(sys.argv[1])
        motor.run()
