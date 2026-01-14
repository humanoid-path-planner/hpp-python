#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
from unit.conftest import create_constraint_graph_setup
from pyhpp.manipulation.constraint_graph_factory import (
    Constraints,
    ConstraintFactory,
)


class TestConstraintFactoryBuildGrasp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.graph, cls.factory, cls.robot, cls.objects = (
            create_constraint_graph_setup()
        )

    def test_build_grasp_returns_dict(self):
        result = self.factory.constraints.buildGrasp(
            "ur3/gripper", "sphere0/handle"
        )

        self.assertIsInstance(result, dict)
        self.assertIn("grasp", result)
        self.assertIn("graspComplement", result)
        self.assertIn("preGrasp", result)

    def test_build_grasp_constraint_values_are_constraints(self):
        result = self.factory.constraints.buildGrasp(
            "ur3/gripper", "sphere0/handle"
        )

        self.assertIsInstance(result["grasp"], Constraints)
        self.assertIsInstance(result["graspComplement"], Constraints)
        self.assertIsInstance(result["preGrasp"], Constraints)

    def test_build_grasp_caches_constraints(self):
        cf = self.factory.constraints

        result1 = cf.buildGrasp("ur3/gripper", "sphere1/handle")
        result2 = cf.buildGrasp("ur3/gripper", "sphere1/handle")

        self.assertEqual(
            result1["grasp"].numConstraints,
            result2["grasp"].numConstraints
        )


class TestConstraintFactoryBuildPlacement(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.graph, cls.factory, cls.robot, cls.objects = (
            create_constraint_graph_setup()
        )

    def test_build_placement_returns_dict(self):
        result = self.factory.constraints.buildPlacement("sphere0")

        self.assertIsInstance(result, dict)
        self.assertIn("placement", result)
        self.assertIn("placementComplement", result)
        self.assertIn("prePlacement", result)

    def test_build_placement_constraint_values_are_constraints(self):
        result = self.factory.constraints.buildPlacement("sphere0")

        self.assertIsInstance(result["placement"], Constraints)
        self.assertIsInstance(result["placementComplement"], Constraints)
        self.assertIsInstance(result["prePlacement"], Constraints)


class TestConstraintFactoryRegistration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.graph, cls.factory, cls.robot, cls.objects = (
            create_constraint_graph_setup()
        )

    def test_constraints_registered_after_build(self):
        cf = self.factory.constraints

        cf.buildGrasp("ur3/gripper", "sphere0/handle")

        name = "ur3/gripper grasps sphere0/handle"
        self.assertIn(name, cf.available_constraints)

    def test_pregrasp_registered_after_build(self):
        cf = self.factory.constraints

        cf.buildGrasp("ur3/gripper", "sphere0/handle")

        name = "ur3/gripper pregrasps sphere0/handle"
        self.assertIn(name, cf.available_constraints)


class TestConstraintFactoryNegativeCases(unittest.TestCase):
    """Negative test cases for ConstraintFactory."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.graph, cls.factory, cls.robot, cls.objects = (
            create_constraint_graph_setup()
        )

    def test_build_grasp_invalid_gripper_raises(self):
        """Building grasp with non-existent gripper should raise."""
        cf = self.factory.constraints

        with self.assertRaises(Exception):
            cf.buildGrasp("nonexistent/gripper", "sphere0/handle")

    def test_build_grasp_invalid_handle_raises(self):
        """Building grasp with non-existent handle should raise."""
        cf = self.factory.constraints

        with self.assertRaises(Exception):
            cf.buildGrasp("ur3/gripper", "nonexistent/handle")

    def test_build_placement_invalid_object_raises(self):
        """Building placement with non-existent object should raise."""
        cf = self.factory.constraints

        with self.assertRaises(Exception):
            cf.buildPlacement("nonexistent_object")

    def test_grasp_constraints_not_empty(self):
        """Built grasp constraints should not be empty."""
        cf = self.factory.constraints

        result = cf.buildGrasp("ur3/gripper", "sphere0/handle")

        self.assertFalse(result["grasp"].empty())

    def test_placement_constraints_not_empty(self):
        """Built placement constraints should not be empty."""
        cf = self.factory.constraints

        result = cf.buildPlacement("sphere0")

        self.assertFalse(result["placement"].empty())


if __name__ == "__main__":
    unittest.main()
