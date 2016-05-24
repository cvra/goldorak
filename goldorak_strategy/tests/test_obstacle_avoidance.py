#!/usr/bin/env python
PKG = 'goldorak_base'

import sys
import unittest

from ..scripts import obstacle_avoidance

## A sample python unit test
class TestObstacleAvoidance(unittest.TestCase):
    def test_not_moving(self):
        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=0, distance=0.1, angle=0, safety_distance=0.3, safety_half_angle=0.5)
        self.assertFalse(res)

        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=-0, distance=0.4, angle=0, safety_distance=0.3, safety_half_angle=0.5)
        self.assertFalse(res)

    def test_obstacle_in_front_moving_forward(self):
        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=0.1, distance=0.1, angle=0, safety_distance=0.3, safety_half_angle=0.5)
        self.assertTrue(res)

    def test_obstacle_in_front_moving_backward(self):
        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=-0.1, distance=0.1, angle=0, safety_distance=0.3, safety_half_angle=0.5)
        self.assertFalse(res)

    def test_obstacle_in_back_moving_forward(self):
        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=0.1, distance=0.1, angle=3.14, safety_distance=0.3, safety_half_angle=0.5)
        self.assertFalse(res)

    def test_obstacle_in_back_moving_backward(self):
        res = obstacle_avoidance.obstacle_within_vision_cone(
            vel_x=-0.1, distance=0.1, angle=3.14, safety_distance=0.3, safety_half_angle=0.5)
        self.assertTrue(res)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_obstacle_avoidance', TestObstacleAvoidance)
