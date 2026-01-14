#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf


UR5_URDF = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
UR5_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"


def create_ur5_device():
    robot = Device("ur5")
    urdf.loadModel(robot, 0, "ur5", "anchor", UR5_URDF, UR5_SRDF, SE3.Identity())
    return robot


class TestDeviceInstantiation(unittest.TestCase):

    def test_device_creates_with_name(self):
        robot = Device("test_robot")

        self.assertEqual(robot.name(), "test_robot")

    def test_device_loads_urdf(self):
        robot = create_ur5_device()

        self.assertEqual(robot.name(), "ur5")
        self.assertGreater(robot.configSize(), 0)


class TestDeviceConfiguration(unittest.TestCase):

    def test_config_size_returns_dof_count(self):
        robot = create_ur5_device()

        size = robot.configSize()

        self.assertEqual(size, 6)

    def test_number_dof_returns_velocity_size(self):
        robot = create_ur5_device()

        ndof = robot.numberDof()

        self.assertEqual(ndof, 6)

    def test_current_configuration_settable(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        result = robot.currentConfiguration(q)

        self.assertTrue(result)

    def test_current_configuration_readable(self):
        robot = create_ur5_device()
        q = np.array([0.1, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)

        current_q = robot.currentConfiguration()

        np.testing.assert_array_almost_equal(current_q, q)


class TestDeviceJointBounds(unittest.TestCase):

    def test_set_joint_bounds(self):
        robot = create_ur5_device()

        robot.setJointBounds("ur5/shoulder_pan_joint", [-1.0, 1.0])

    def test_set_joint_bounds_multiple_dof(self):
        robot = create_ur5_device()

        robot.setJointBounds("ur5/shoulder_pan_joint", [-2.0, 2.0])

    def test_set_joint_bounds_invalid_size_raises(self):
        robot = create_ur5_device()

        with self.assertRaises(Exception):
            robot.setJointBounds("ur5/shoulder_pan_joint", [-1.0])


class TestDeviceForwardKinematics(unittest.TestCase):

    def test_compute_forward_kinematics_joint_position(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)

        robot.computeForwardKinematics(1)

    def test_compute_frames_forward_kinematics(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)
        robot.computeForwardKinematics(1)

        robot.computeFramesForwardKinematics()

    def test_get_joint_position_returns_7_elements(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)
        robot.computeForwardKinematics(1)
        robot.computeFramesForwardKinematics()

        position = robot.getJointPosition("ur5/ee_fixed_joint")

        self.assertEqual(len(position), 7)

    def test_joint_position_format(self):
        """Joint position should be [x, y, z, qx, qy, qz, qw]."""
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)
        robot.computeForwardKinematics(1)
        robot.computeFramesForwardKinematics()

        position = robot.getJointPosition("ur5/ee_fixed_joint")

        quat_norm = sum(x**2 for x in position[3:7])
        self.assertAlmostEqual(quat_norm, 1.0, places=5)

    def test_get_joints_position_batch(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        joints = ["ur5/shoulder_link", "ur5/ee_fixed_joint"]

        positions = robot.getJointsPosition(q, joints)

        self.assertEqual(len(positions), 2)
        for pos in positions:
            self.assertEqual(len(pos), 7)


class TestDeviceRankInConfiguration(unittest.TestCase):

    def test_rank_in_configuration_returns_dict(self):
        robot = create_ur5_device()

        ranks = robot.rankInConfiguration

        self.assertIsInstance(ranks, dict)

    def test_rank_in_configuration_has_joints(self):
        robot = create_ur5_device()

        ranks = robot.rankInConfiguration

        self.assertIn("ur5/shoulder_pan_joint", ranks)
        self.assertIn("ur5/shoulder_lift_joint", ranks)

    def test_rank_values_are_valid_indices(self):
        robot = create_ur5_device()

        ranks = robot.rankInConfiguration

        for joint_name, rank in ranks.items():
            self.assertGreaterEqual(rank, 0)
            self.assertLess(rank, robot.configSize())


class TestDeviceNegativeCases(unittest.TestCase):

    def test_get_joint_position_invalid_frame_raises(self):
        robot = create_ur5_device()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        robot.currentConfiguration(q)
        robot.computeForwardKinematics(1)
        robot.computeFramesForwardKinematics()

        with self.assertRaises(Exception):
            robot.getJointPosition("nonexistent_joint")

    def test_set_joint_bounds_invalid_joint_raises(self):
        robot = create_ur5_device()

        with self.assertRaises(Exception):
            robot.setJointBounds("nonexistent_joint", [-1.0, 1.0])


if __name__ == "__main__":
    unittest.main()
