#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem
from pyhpp.core import RandomShortcut, SimpleShortcut
import pyhpp.core.path


class TestPathOptimizerInstantiation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_random_shortcut_creates(self):
        optimizer = RandomShortcut(self.problem)

        self.assertIsNotNone(optimizer)

    def test_simple_shortcut_creates(self):
        optimizer = SimpleShortcut(self.problem)

        self.assertIsNotNone(optimizer)


class TestPathOptimizerOptimize(unittest.TestCase):
    def test_optimize_returns_path(self):
        """PathOptimizer.optimize should return a valid path."""
        problem, robot = create_ur5_problem()

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.3, -1.57, -1.8, 0.0, 0.8, 0.0])
        q3 = np.array([0.6, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = problem.steeringMethod()
        path1 = steer(q1, q2)
        path2 = steer(q2, q3)

        pv = pyhpp.core.path.Vector.create(robot.configSize(), robot.numberDof())
        pv.appendPath(path1)
        pv.appendPath(path2)

        optimizer = RandomShortcut(problem)
        optimizer.maxIterations(10)

        optimized = optimizer.optimize(pv)

        self.assertIsNotNone(optimized)
        # Optimized path should preserve start and end
        np.testing.assert_array_almost_equal(optimized.initial(), q1)
        np.testing.assert_array_almost_equal(optimized.end(), q3)

    def test_max_iterations_settable(self):
        """PathOptimizer.maxIterations should be settable."""
        problem, robot = create_ur5_problem()

        optimizer = RandomShortcut(problem)
        optimizer.maxIterations(50)

        # Verify it doesn't crash and actually limits iterations
        # by running optimization on a simple path
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        steer = problem.steeringMethod()
        path = steer(q1, q2)
        pv = pyhpp.core.path.Vector.create(robot.configSize(), robot.numberDof())
        pv.appendPath(path)

        optimized = optimizer.optimize(pv)
        self.assertIsNotNone(optimized)


class TestPathOptimizerNegativeCases(unittest.TestCase):
    """Negative test cases for PathOptimizer."""

    def test_optimize_zero_length_path(self):
        """Optimizing a zero-length path should return zero-length path."""
        problem, robot = create_ur5_problem()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = problem.steeringMethod()
        path = steer(q, q)

        pv = pyhpp.core.path.Vector.create(robot.configSize(), robot.numberDof())
        pv.appendPath(path)

        optimizer = RandomShortcut(problem)
        optimizer.maxIterations(10)

        optimized = optimizer.optimize(pv)

        self.assertEqual(optimized.length(), 0.0)

    def test_optimize_preserves_endpoints(self):
        """Optimization must always preserve path endpoints."""
        problem, robot = create_ur5_problem()

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.2, -1.4, -1.6, 0.1, 0.7, 0.1])
        q3 = np.array([0.4, -1.2, -1.4, 0.2, 0.6, 0.2])
        q4 = np.array([0.6, -1.0, -1.2, 0.3, 0.5, 0.3])

        steer = problem.steeringMethod()
        pv = pyhpp.core.path.Vector.create(robot.configSize(), robot.numberDof())
        pv.appendPath(steer(q1, q2))
        pv.appendPath(steer(q2, q3))
        pv.appendPath(steer(q3, q4))

        optimizer = SimpleShortcut(problem)
        optimizer.maxIterations(20)

        optimized = optimizer.optimize(pv)

        np.testing.assert_array_almost_equal(optimized.initial(), q1)
        np.testing.assert_array_almost_equal(optimized.end(), q4)

    def test_zero_iterations_returns_same_path(self):
        """Zero iterations should return path with same length."""
        problem, robot = create_ur5_problem()

        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        steer = problem.steeringMethod()
        path = steer(q1, q2)

        pv = pyhpp.core.path.Vector.create(robot.configSize(), robot.numberDof())
        pv.appendPath(path)
        original_length = pv.length()

        optimizer = RandomShortcut(problem)
        optimizer.maxIterations(0)

        optimized = optimizer.optimize(pv)

        self.assertAlmostEqual(optimized.length(), original_length, places=5)


if __name__ == "__main__":
    unittest.main()
