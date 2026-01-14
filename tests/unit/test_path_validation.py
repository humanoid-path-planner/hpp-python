#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem


class TestPathValidation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_collision_free_path_is_valid(self):
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.1, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        is_valid, valid_part, report = self.problem.pathValidation().validate(
            path, False
        )
        self.assertTrue(is_valid)

    def test_zero_length_path_returns_valid_part(self):
        """A zero-length path (same start/end) should still return a valid_part."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q1)

        is_valid, valid_part, report = self.problem.pathValidation().validate(
            path, False
        )
        self.assertIsNotNone(valid_part)
        self.assertEqual(valid_part.length(), 0.0)

    def test_validate_returns_three_values(self):
        """Validate always returns (bool, path, report) tuple."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        result = self.problem.pathValidation().validate(path, False)
        self.assertEqual(len(result), 3)


class TestPathValidationNegativeCases(unittest.TestCase):
    """Negative test cases for PathValidation."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_valid_part_never_none(self):
        """Valid part should never be None, even for invalid paths."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        is_valid, valid_part, report = self.problem.pathValidation().validate(
            path, False
        )

        self.assertIsNotNone(valid_part)

    def test_valid_part_length_less_than_or_equal_original(self):
        """Valid part length should never exceed original path length."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        is_valid, valid_part, report = self.problem.pathValidation().validate(
            path, False
        )

        self.assertLessEqual(valid_part.length(), path.length())

    def test_reverse_flag_produces_valid_result(self):
        """Validation with reverse=True should still produce valid results."""
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.3, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        is_valid, valid_part, report = self.problem.pathValidation().validate(
            path, True
        )

        self.assertIsNotNone(valid_part)
        self.assertIsInstance(is_valid, bool)


if __name__ == "__main__":
    unittest.main()
