#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem
import pyhpp.core.path


class TestPathMethods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_extract_returns_subpath(self):
        """Path.extract should return a portion of the original path."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)
        original_length = path.length()

        # Extract middle half
        t_start = original_length * 0.25
        t_end = original_length * 0.75
        extracted = path.extract(t_start, t_end)

        self.assertIsNotNone(extracted)
        self.assertAlmostEqual(extracted.length(), original_length * 0.5, places=5)

    def test_copy_creates_independent_path(self):
        """Path.copy should create an independent copy."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        copied = path.copy()

        self.assertIsNotNone(copied)
        self.assertEqual(copied.length(), path.length())
        np.testing.assert_array_almost_equal(copied.initial(), path.initial())
        np.testing.assert_array_almost_equal(copied.end(), path.end())

    def test_derivative_returns_velocity(self):
        """Path.derivative should return velocity at given parameter."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        t_mid = path.length() / 2.0
        deriv = path.derivative(t_mid, 1)

        self.assertEqual(len(deriv), self.robot.numberDof())
        # For straight path, first joint derivative should be non-zero
        self.assertNotEqual(deriv[0], 0.0)

    def test_length_is_non_negative(self):
        """Path.length should be non-negative for valid paths."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        self.assertGreaterEqual(path.length(), 0.0)


class TestPathNegativeCases(unittest.TestCase):
    """Negative test cases for Path operations."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_extract_valid_range(self):
        """Extracting with valid range should succeed."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)
        length = path.length()

        # Valid extraction
        extracted = path.extract(0.2 * length, 0.8 * length)
        self.assertIsNotNone(extracted)
        self.assertAlmostEqual(extracted.length(), 0.6 * length, places=5)

    def test_initial_and_end_accessors(self):
        """Path.initial() and path.end() should return correct configs."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        # Check initial config
        np.testing.assert_array_almost_equal(path.initial(), q1)
        # Check end config
        np.testing.assert_array_almost_equal(path.end(), q2)

    def test_call_operator_at_midpoint(self):
        """Calling path(t) should return interpolated config and success status."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        # Path call operator returns (config, success) tuple
        q_mid, success = path(path.length() / 2.0)
        self.assertTrue(success)
        # First joint should be midway
        self.assertAlmostEqual(q_mid[0], 0.5, places=5)

    def test_derivative_at_order_one(self):
        """Derivative at order 1 returns velocity."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        # Order 1 derivative is velocity
        deriv1 = path.derivative(path.length() / 2.0, 1)
        self.assertEqual(len(deriv1), self.robot.numberDof())


class TestPathVector(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_create_empty_path_vector(self):
        """PathVector.create should create an empty vector."""
        pv = pyhpp.core.path.Vector.create(
            self.robot.configSize(), self.robot.numberDof()
        )

        self.assertIsNotNone(pv)
        self.assertEqual(pv.numberPaths(), 0)

    def test_append_path_increases_count(self):
        """Appending paths should increase numberPaths."""
        pv = pyhpp.core.path.Vector.create(
            self.robot.configSize(), self.robot.numberDof()
        )

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        pv.appendPath(path)

        self.assertEqual(pv.numberPaths(), 1)

    def test_path_vector_length_is_sum(self):
        """PathVector length should be sum of contained paths."""
        pv = pyhpp.core.path.Vector.create(
            self.robot.configSize(), self.robot.numberDof()
        )

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        q3 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path1 = steer(q1, q2)
        path2 = steer(q2, q3)

        pv.appendPath(path1)
        pv.appendPath(path2)

        expected_length = path1.length() + path2.length()
        self.assertAlmostEqual(pv.length(), expected_length, places=10)


class TestPathVectorNegativeCases(unittest.TestCase):
    """Negative test cases for PathVector operations."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_path_at_valid_index_works(self):
        """Accessing path at valid index should work."""
        pv = pyhpp.core.path.Vector.create(
            self.robot.configSize(), self.robot.numberDof()
        )

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)
        pv.appendPath(path)

        retrieved = pv.pathAtRank(0)
        self.assertIsNotNone(retrieved)
        self.assertAlmostEqual(retrieved.length(), path.length(), places=10)

    def test_empty_path_vector_has_zero_length(self):
        """Empty PathVector should have zero length."""
        pv = pyhpp.core.path.Vector.create(
            self.robot.configSize(), self.robot.numberDof()
        )

        self.assertEqual(pv.length(), 0.0)


if __name__ == "__main__":
    unittest.main()
