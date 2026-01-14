#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pinocchio import SE3, StdVec_Bool as Mask
from pyhpp.pinocchio import Device, LiegroupElement, urdf
from pyhpp.constraints import Position


UR5_URDF = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
UR5_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"


def create_ur5_device():
    robot = Device("ur5")
    urdf.loadModel(robot, 0, "ur5", "anchor", UR5_URDF, UR5_SRDF, SE3.Identity())
    return robot


class TestPositionConstraint(unittest.TestCase):
    def test_create_position_constraint(self):
        robot = create_ur5_device()
        joint_id = robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "test_position", robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        self.assertIsNotNone(pc)

    def test_position_constraint_str(self):
        robot = create_ur5_device()
        joint_id = robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "my_position", robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        self.assertIn("my_position", str(pc))

    def test_position_constraint_dimensions(self):
        robot = create_ur5_device()
        joint_id = robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position("position", robot, joint_id, SE3.Identity(), SE3.Identity(), mask)

        self.assertEqual(pc.ndo, 3)


class TestPositionConstraintEvaluation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = create_ur5_device()

    def test_value_cpp_api(self):
        joint_id = self.robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "position", self.robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        q = np.zeros((pc.ni, 1))
        v = LiegroupElement(pc.outputSpace())
        pc.value(v, q)

        self.assertEqual(len(v.vector()), 3)

    def test_jacobian_cpp_api(self):
        joint_id = self.robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "position", self.robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        q = self.robot.currentConfiguration()
        J = np.zeros((pc.ndo, pc.ndi))
        pc.jacobian(J, q)

        self.assertEqual(J.shape, (3, self.robot.numberDof()))

    def test_call_operator(self):
        joint_id = self.robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "position", self.robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        q = np.zeros((pc.ni, 1))
        result = pc(q)

        self.assertIsNotNone(result)

    def test_jacobian_shorthand(self):
        joint_id = self.robot.model().getJointId("ur5/wrist_3_joint")
        mask = Mask()
        mask[:] = (True, True, True)

        pc = Position(
            "position", self.robot, joint_id, SE3.Identity(), SE3.Identity(), mask
        )

        q = self.robot.currentConfiguration()
        J = pc.J(q)

        self.assertEqual(J.shape[0], pc.ndo)


if __name__ == "__main__":
    unittest.main()
