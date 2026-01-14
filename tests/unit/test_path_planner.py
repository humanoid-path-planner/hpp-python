#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem
from pyhpp.core import (
    DiffusingPlanner,
    BiRRTPlanner,
    VisibilityPrmPlanner,
)


class TestPathPlannerInstantiation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_diffusing_planner_creates(self):
        planner = DiffusingPlanner(self.problem)

        self.assertIsNotNone(planner)

    def test_birrt_planner_creates(self):
        planner = BiRRTPlanner(self.problem)

        self.assertIsNotNone(planner)

    def test_visibility_prm_planner_creates(self):
        planner = VisibilityPrmPlanner(self.problem)

        self.assertIsNotNone(planner)


class TestPathPlannerConfiguration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_max_iterations_settable(self):
        planner = DiffusingPlanner(self.problem)

        planner.maxIterations(500)

        self.assertEqual(planner.maxIterations(), 500)

    def test_max_iterations_defaults_to_zero(self):
        planner = BiRRTPlanner(self.problem)

        self.assertEqual(planner.maxIterations(), 0)


class TestPathPlannerSolve(unittest.TestCase):

    def test_birrt_solves_simple_path(self):
        """BiRRT should find a path between nearby collision-free configurations."""
        problem, robot = create_ur5_problem()

        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_goal = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        problem.initConfig(q_init)
        problem.addGoalConfig(q_goal)

        planner = BiRRTPlanner(problem)
        planner.maxIterations(200)

        path = planner.solve()

        self.assertIsNotNone(path)
        self.assertGreater(path.length(), 0)
        np.testing.assert_array_almost_equal(path.initial(), q_init)
        np.testing.assert_array_almost_equal(path.end(), q_goal)


class TestPathPlannerNegativeCases(unittest.TestCase):
    """Negative test cases for path planners."""

    def test_solve_without_goal_raises(self):
        """Solving without a goal configuration should raise an error."""
        problem, robot = create_ur5_problem()
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        problem.initConfig(q_init)

        planner = BiRRTPlanner(problem)
        planner.maxIterations(10)

        with self.assertRaises(Exception):
            planner.solve()

    def test_solve_without_init_raises(self):
        """Solving without an initial configuration should raise an error."""
        problem, robot = create_ur5_problem()
        q_goal = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        problem.addGoalConfig(q_goal)

        planner = BiRRTPlanner(problem)
        planner.maxIterations(10)

        with self.assertRaises(Exception):
            planner.solve()

    def test_max_iterations_zero_limits_exploration(self):
        """Planner with zero max iterations should do minimal exploration."""
        problem, robot = create_ur5_problem()
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_goal = np.array([3.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        problem.initConfig(q_init)
        problem.addGoalConfig(q_goal)

        planner = BiRRTPlanner(problem)
        planner.maxIterations(0)

        try:
            planner.solve()
        except Exception:
            pass

        roadmap = planner.roadmap()
        self.assertLessEqual(len(roadmap.nodes()), 2)

if __name__ == "__main__":
    unittest.main()
