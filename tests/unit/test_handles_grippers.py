#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
from pinocchio import SE3
from pyhpp.manipulation import Device, urdf


UR3_URDF = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
UR3_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
SPHERE_URDF = "package://hpp_environments/urdf/construction_set/sphere.urdf"
SPHERE_SRDF = "package://hpp_environments/srdf/construction_set/sphere.srdf"


def create_manipulation_device():
    robot = Device("ur3-sphere")
    urdf.loadModel(robot, 0, "ur3", "anchor", UR3_URDF, UR3_SRDF, SE3.Identity())
    urdf.loadModel(
        robot, 0, "sphere", "freeflyer", SPHERE_URDF, SPHERE_SRDF, SE3.Identity()
    )
    return robot


class TestGrippers(unittest.TestCase):
    def test_grippers_returns_map(self):
        robot = create_manipulation_device()

        grippers = robot.grippers()

        self.assertIsNotNone(grippers)

    def test_gripper_exists(self):
        robot = create_manipulation_device()

        grippers = robot.grippers()

        self.assertIn("ur3/gripper", grippers)

    def test_gripper_has_local_position(self):
        robot = create_manipulation_device()
        grippers = robot.grippers()

        gripper = grippers["ur3/gripper"]

        self.assertTrue(hasattr(gripper, "localPosition"))


class TestHandles(unittest.TestCase):
    def test_handles_returns_map(self):
        robot = create_manipulation_device()

        handles = robot.handles()

        self.assertIsNotNone(handles)

    def test_handle_exists(self):
        robot = create_manipulation_device()

        handles = robot.handles()

        self.assertIn("sphere/handle", handles)

    def test_handle_has_local_position(self):
        robot = create_manipulation_device()
        handles = robot.handles()

        handle = handles["sphere/handle"]

        self.assertTrue(hasattr(handle, "localPosition"))

    def test_handle_has_mask(self):
        robot = create_manipulation_device()
        handles = robot.handles()

        handle = handles["sphere/handle"]

        self.assertTrue(hasattr(handle, "mask"))

    def test_handle_mask_modifiable(self):
        robot = create_manipulation_device()
        handles = robot.handles()
        handle = handles["sphere/handle"]

        handle.mask = [True, True, True, False, True, True]

        self.assertEqual(handle.mask[3], False)


class TestGraspCreation(unittest.TestCase):
    def test_create_grasp_constraint(self):
        robot = create_manipulation_device()
        handles = robot.handles()
        grippers = robot.grippers()

        handle = handles["sphere/handle"]
        gripper = grippers["ur3/gripper"]

        constraint = handle.createGrasp(gripper, "test_grasp")

        self.assertIsNotNone(constraint)


if __name__ == "__main__":
    unittest.main()
