#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem


class TestSteeringMethod(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_creates_path_with_correct_endpoints(self):
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([1.57, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        self.assertIsNotNone(path)
        self.assertGreater(path.length(), 0)
        np.testing.assert_array_almost_equal(path.initial(), q1)
        np.testing.assert_array_almost_equal(path.end(), q2)

    def test_path_length_is_positive(self):
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.0, -1.5, 0.2, 0.5, 0.1])

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)

        self.assertGreater(path.length(), 0)

    def test_identical_configs_creates_zero_length_path(self):
        """Steering between identical configs should create a zero-length path."""
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = self.problem.steeringMethod()
        path = steer(q, q)

        self.assertIsNotNone(path)
        self.assertEqual(path.length(), 0.0)


if __name__ == "__main__":
    unittest.main()
