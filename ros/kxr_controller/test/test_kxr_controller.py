#!/usr/bin/env python

from __future__ import print_function

import unittest

from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
from numpy import testing
import rospy
from skrobot.model import RobotModel


PKG = 'kxr_controller'
NAME = 'test_kxr_controller'


class TestKXRController(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)
        rospy.sleep(3.0)
        namespace = rospy.get_param('~namespace', None)
        namespace = '/{}'.format(namespace) if namespace else ''
        self.robot_model = RobotModel()
        self.robot_model.load_urdf_from_robot_description(
            namespace + '/robot_description')
        self.ri = KXRROSRobotInterface(self.robot_model,
                                       namespace=namespace,
                                       controller_timeout=60.0)

    def test_follow_joint_trajectory(self):
        self.ri.servo_on()
        self.ri.angle_vector(self.robot_model.init_pose(), 1.0)
        self.ri.wait_interpolation()
        self.robot_model.HEAD_JOINT0.joint_angle(np.deg2rad(45))
        self.robot_model.HEAD_JOINT1.joint_angle(np.deg2rad(45))
        self.ri.angle_vector(self.robot_model.angle_vector(), 1.0)
        self.ri.wait_interpolation()
        rospy.sleep(1.0)
        current_angles = self.ri.angle_vector()
        testing.assert_array_almost_equal(
            np.rad2deg(current_angles), [45, 45],
            decimal=0)
        self.ri.angle_vector(self.robot_model.init_pose(), 1.0)
        self.ri.wait_interpolation()
        self.ri.servo_off()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestKXRController)
