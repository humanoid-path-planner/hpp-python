#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem
from pyhpp.core import (
    Progressive,
    GlobalProjector,
)


class TestPathProjectorInstantiation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_progressive_projector_creates(self):
        projector = Progressive(self.robot, 0.1)

        self.assertIsNotNone(projector)

    def test_global_projector_creates(self):
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()
        projector = GlobalProjector(distance, steer, 0.1)

        self.assertIsNotNone(projector)


class TestPathProjectorProject(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_global_projector_projects_unconstrained_path(self):
        """Without constraints, projection should succeed and preserve endpoints."""
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()
        projector = GlobalProjector(distance, steer, 0.1)

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        path = steer(q1, q2)

        success, projected_path = projector.apply(path)

        self.assertTrue(success)
        self.assertIsNotNone(projected_path)
        np.testing.assert_array_almost_equal(projected_path.initial(), q1)
        np.testing.assert_array_almost_equal(projected_path.end(), q2)


class TestPathProjectorNegativeCases(unittest.TestCase):
    """Negative test cases for PathProjector."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_zero_step_creates_projector(self):
        """Projector with zero step should still be created."""
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()

        projector = GlobalProjector(distance, steer, 0.0)

        self.assertIsNotNone(projector)

    def test_negative_step_creates_projector(self):
        """Projector with negative step still creates (library accepts it)."""
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()

        projector = GlobalProjector(distance, steer, -0.1)

        self.assertIsNotNone(projector)

    def test_zero_length_path_projection(self):
        """Projection of zero-length path should succeed."""
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()
        projector = GlobalProjector(distance, steer, 0.1)

        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        path = steer(q, q)

        success, projected_path = projector.apply(path)

        self.assertTrue(success)
        self.assertEqual(projected_path.length(), 0.0)

    def test_apply_returns_two_values(self):
        """Apply should return (success, path) tuple."""
        steer = self.problem.steeringMethod()
        distance = self.problem.distance()
        projector = GlobalProjector(distance, steer, 0.1)

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        path = steer(q1, q2)

        result = projector.apply(path)

        self.assertEqual(len(result), 2)
        self.assertIsInstance(result[0], bool)


if __name__ == "__main__":
    unittest.main()
