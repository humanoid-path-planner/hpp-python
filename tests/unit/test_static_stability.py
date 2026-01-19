#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pinocchio import SE3

from pyhpp.manipulation import Device, Problem, urdf
from pyhpp.core.static_stability_constraint_factory import (
    StaticStabilityConstraintsFactory,
)


def load_romeo():
    romeo_urdf = "package://example-robot-data/robots/romeo_description/urdf/romeo.urdf"
    romeo_srdf = "package://example-robot-data/robots/romeo_description/srdf/romeo_moveit.srdf"

    robot = Device("romeo")
    romeo_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "romeo", "freeflyer", romeo_urdf, romeo_srdf, romeo_pose)

    robot.setJointBounds(
        "romeo/root_joint",
        [-1, 1, -1, 1, 0, 2, -2.0, 2, -2.0, 2, -2.0, 2, -2.0, 2],
    )

    return robot


class TestStaticStabilityConstraintsFactory(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = load_romeo()
        cls.problem = Problem(cls.robot)
        cls.factory = StaticStabilityConstraintsFactory(cls.problem, cls.robot)
        cls.q0 = cls.robot.currentConfiguration()
        cls.left_ankle = "romeo/LAnkleRoll"
        cls.right_ankle = "romeo/RAnkleRoll"

    def test_create_static_stability_constraint_returns_dict(self):
        constraints = self.factory.createStaticStabilityConstraint(
            "test_", "", self.left_ankle, self.right_ankle, self.q0
        )

        self.assertIsInstance(constraints, dict)
        self.assertIn("test_relative-com", constraints)
        self.assertIn("test_pose-left-foot", constraints)
        self.assertIn("test_pose-right-foot", constraints)
        self.assertEqual(len(constraints), 3)

    def test_create_sliding_stability_constraint_returns_dict(self):
        constraints = self.factory.createSlidingStabilityConstraint(
            "slide_", "", self.left_ankle, self.right_ankle, self.q0
        )

        self.assertIsInstance(constraints, dict)
        self.assertIn("slide_relative-com", constraints)
        self.assertIn("slide_relative-pose", constraints)
        self.assertIn("slide_pose-left-foot", constraints)
        self.assertIn("slide_pose-left-foot-complement", constraints)
        self.assertEqual(len(constraints), 4)

    def test_create_aligned_com_stability_constraint_returns_dict(self):
        constraints = self.factory.createAlignedCOMStabilityConstraint(
            "aligned_", "", self.left_ankle, self.right_ankle, self.q0, sliding=False
        )

        self.assertIsInstance(constraints, dict)
        self.assertIn("aligned_com-between-feet", constraints)
        self.assertIn("aligned_pose-left-foot", constraints)
        self.assertIn("aligned_pose-right-foot", constraints)
        self.assertEqual(len(constraints), 3)

    def test_create_aligned_com_stability_constraint_sliding(self):
        constraints = self.factory.createAlignedCOMStabilityConstraint(
            "aligned_slide_", "", self.left_ankle, self.right_ankle, self.q0, sliding=True
        )

        self.assertIsInstance(constraints, dict)
        self.assertEqual(len(constraints), 3)


if __name__ == "__main__":
    unittest.main()
