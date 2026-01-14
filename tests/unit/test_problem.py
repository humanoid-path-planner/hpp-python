#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pinocchio import SE3
from unit.conftest import create_ur5_problem

class TestProblemAccessors(unittest.TestCase):

    def test_steering_method_returns_object(self):
        problem, robot = create_ur5_problem()

        sm = problem.steeringMethod()

        self.assertIsNotNone(sm)

    def test_distance_returns_object(self):
        problem, robot = create_ur5_problem()

        distance = problem.distance()

        self.assertIsNotNone(distance)

    def test_configuration_shooter_returns_object(self):
        problem, robot = create_ur5_problem()

        shooter = problem.configurationShooter()

        self.assertIsNotNone(shooter)

    def test_path_validation_returns_object(self):
        problem, robot = create_ur5_problem()

        pv = problem.pathValidation()

        self.assertIsNotNone(pv)

    def test_config_validation_returns_object(self):
        problem, robot = create_ur5_problem()

        cv = problem.configValidation()

        self.assertIsNotNone(cv)


class TestProblemDirectPath(unittest.TestCase):

    def test_direct_path_without_validation(self):
        problem, robot = create_ur5_problem()
        q_start = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_end = np.array([0.1, -1.57, -1.8, 0.0, 0.8, 0.0])

        result = problem.directPath(q_start, q_end, False)

        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 3)
        valid, path, report = result
        self.assertTrue(valid)
        self.assertIsNotNone(path)

    def test_direct_path_with_validation(self):
        problem, robot = create_ur5_problem()
        problem.addConfigValidation("JointBoundValidation")
        q_start = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_end = np.array([0.1, -1.57, -1.8, 0.0, 0.8, 0.0])

        valid, path, report = problem.directPath(q_start, q_end, True)

        self.assertIsInstance(valid, bool)

    def test_direct_path_endpoints_match(self):
        problem, robot = create_ur5_problem()
        q_start = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_end = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        valid, path, report = problem.directPath(q_start, q_end, False)

        np.testing.assert_array_almost_equal(path.initial(), q_start)
        np.testing.assert_array_almost_equal(path.end(), q_end)


class TestProblemConstraintProjection(unittest.TestCase):

    def test_error_threshold_settable(self):
        problem, robot = create_ur5_problem()

        problem.errorThreshold = 1e-6

        self.assertAlmostEqual(problem.errorThreshold, 1e-6)

    def test_max_iter_projection_settable(self):
        problem, robot = create_ur5_problem()

        problem.maxIterProjection = 50

        self.assertEqual(problem.maxIterProjection, 50)

    def test_apply_constraints_returns_tuple(self):
        problem, robot = create_ur5_problem()
        q = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        result = problem.applyConstraints(q)

        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 3)
        success, output, residual = result
        self.assertIsInstance(success, bool)
        self.assertEqual(len(output), len(q))


class TestProblemTransformationConstraints(unittest.TestCase):

    def test_create_transformation_constraint(self):
        problem, robot = create_ur5_problem()
        M = SE3.Identity()
        mask = [True, True, True, True, True, True]

        constraint = problem.createTransformationConstraint(
            "test_constraint",
            "",
            "ur5/ee_fixed_joint",
            M,
            mask
        )

        self.assertIsNotNone(constraint)

    def test_create_relative_transformation_constraint(self):
        problem, robot = create_ur5_problem()
        M = SE3.Identity()
        mask = [True, True, True, True, True, True]

        constraint = problem.createTransformationConstraint(
            "relative_constraint",
            "ur5/shoulder_link",
            "ur5/ee_fixed_joint",
            M,
            mask
        )

        self.assertIsNotNone(constraint)


if __name__ == "__main__":
    unittest.main()
