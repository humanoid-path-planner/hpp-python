#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
from unit.conftest import (
    create_security_margins_instance,
    create_constraint_graph_setup,
)
from pyhpp.manipulation.security_margins import SecurityMargins


class TestSecurityMarginsCorrectness(unittest.TestCase):
    def test_gripper_correctly_mapped_to_robot(self):
        sm, *_ = create_security_margins_instance()

        self.assertIn("ur3/gripper", sm.gripperToRobot)
        self.assertEqual(sm.gripperToRobot["ur3/gripper"], "ur3")
        self.assertIn("ur3/gripper", sm.gripperToJoints)
        self.assertGreater(len(sm.gripperToJoints["ur3/gripper"]), 0)

    def test_margins_stored_symmetrically(self):
        sm, *_ = create_security_margins_instance()

        sm.setSecurityMarginBetween("ur3", "sphere0", 0.05)

        self.assertEqual(sm.getSecurityMarginBetween("ur3", "sphere0"), 0.05)
        self.assertEqual(sm.getSecurityMarginBetween("sphere0", "ur3"), 0.05)

    def test_default_margin_used_for_unset_pairs(self):
        sm, *_ = create_security_margins_instance()

        sm.defaultMargin = 0.03
        self.assertEqual(sm.getSecurityMarginBetween("sphere0", "sphere1"), 0.03)

    def test_apply_sets_margins_on_edges(self):
        sm, _, graph, *_ = create_security_margins_instance()

        sm.defaultMargin = 0.02
        sm.setSecurityMarginBetween("ur3", "sphere0", 0.05)
        sm.apply()

        edges = graph.getTransitions()
        self.assertGreater(len(edges), 0)

        for edge in edges:
            matrix = graph.getSecurityMarginMatrixForTransition(edge)
            self.assertIsNotNone(matrix)

    def test_full_workflow_with_graph(self):
        problem, graph, factory, robot, objects = create_constraint_graph_setup()

        sm = SecurityMargins(problem, factory, ["ur3"] + objects, robot)
        sm.defaultMargin = 0.02
        sm.setSecurityMarginBetween("ur3", "sphere0", 0.05)
        sm.setSecurityMarginBetween("ur3", "sphere1", 0.05)
        sm.apply()

        edges = graph.getTransitions()
        self.assertGreater(len(edges), 0)

        state = graph.getState("free")
        q = robot.currentConfiguration()
        result, _, _ = graph.applyStateConstraints(state, q)
        self.assertIsInstance(result, bool)

    def test_grasp_edge_has_zero_margin_between_gripper_and_object(self):
        """
        Grasp edges must have zero margin between gripper and grasped object,
        otherwise collision detection would always reject the grasp.
        Free/transit edges keep the configured safety margin.
        """
        problem, graph, factory, robot, objects = create_constraint_graph_setup()

        sm = SecurityMargins(problem, factory, ["ur3"] + objects, robot)
        configured_margin = 0.05
        sm.defaultMargin = configured_margin
        sm.setSecurityMarginBetween("ur3", "sphere0", configured_margin)
        sm.apply()

        model = robot.model()
        gripper_idx = model.getJointId("ur3/wrist_3_joint")
        object_idx = model.getJointId("sphere0/root_joint")

        def get_margin(edge, idx1, idx2):
            matrix = graph.getSecurityMarginMatrixForTransition(edge)
            if matrix and len(matrix) > idx1 and len(matrix[idx1]) > idx2:
                return matrix[idx1][idx2]
            return None

        edges = graph.getTransitions()
        edge_names = graph.getTransitionNames()

        found_free_loop = False
        found_grasp_edge = False
        free_margin = None
        grasp_margin = None

        for edge, name in zip(edges, edge_names):
            margin = get_margin(edge, gripper_idx, object_idx)
            if margin is None:
                continue

            if name == "Loop | f":
                found_free_loop = True
                free_margin = margin
                self.assertAlmostEqual(margin, configured_margin, places=6)

            elif name == "Loop | 0-0":
                found_grasp_edge = True
                grasp_margin = margin
                self.assertEqual(margin, 0.0)

        self.assertTrue(found_free_loop, "Did not find 'Loop | f' edge")
        self.assertTrue(
            found_grasp_edge, f"Did not find 'Loop | 0-0' edge. Available: {edge_names}"
        )

        if free_margin is not None and grasp_margin is not None:
            self.assertNotEqual(free_margin, grasp_margin)

    def test_joint_to_robot_mapping_correct(self):
        sm, *_ = create_security_margins_instance()

        self.assertIn("ur3", sm.robotToJoints)
        self.assertIn("sphere0", sm.robotToJoints)
        self.assertIn("sphere1", sm.robotToJoints)

        for joint in sm.robotToJoints["ur3"]:
            self.assertEqual(sm.jointToRobot[joint], "ur3")

        for joint in sm.robotToJoints["sphere0"]:
            self.assertEqual(sm.jointToRobot[joint], "sphere0")

    def test_get_active_constraints_returns_dict(self):
        sm, _, graph, *_ = create_security_margins_instance()

        edges = graph.getTransitions()
        self.assertGreater(len(edges), 0)

        result = sm.getActiveConstraintsAlongEdge(edges[0])

        self.assertIsInstance(result, dict)
        self.assertIn("place", result)
        self.assertIn("grasp", result)
        self.assertIsInstance(result["place"], list)
        self.assertIsInstance(result["grasp"], list)

    def test_grasp_edge_has_grasp_constraint(self):
        problem, graph, factory, robot, objects = create_constraint_graph_setup()
        sm = SecurityMargins(problem, factory, ["ur3"] + objects, robot)

        edge_names = graph.getTransitionNames()
        edges = graph.getTransitions()

        for edge, name in zip(edges, edge_names):
            if "0-0" in name:
                result = sm.getActiveConstraintsAlongEdge(edge)
                self.assertGreater(len(result["grasp"]), 0)
                break


class TestSecurityMarginsNegativeCases(unittest.TestCase):
    """Negative test cases for SecurityMargins."""

    def test_get_margin_unknown_robot_returns_default(self):
        """Getting margin for unknown robot pair should return default."""
        sm, *_ = create_security_margins_instance()
        sm.defaultMargin = 0.01

        margin = sm.getSecurityMarginBetween("unknown1", "unknown2")
        self.assertEqual(margin, 0.01)

    def test_set_negative_margin_allowed(self):
        """Negative margins should be allowed (for penetration tolerance)."""
        sm, *_ = create_security_margins_instance()

        sm.setSecurityMarginBetween("ur3", "sphere0", -0.01)

        self.assertEqual(sm.getSecurityMarginBetween("ur3", "sphere0"), -0.01)

    def test_zero_default_margin(self):
        """Zero default margin should work correctly."""
        sm, *_ = create_security_margins_instance()
        sm.defaultMargin = 0.0

        self.assertEqual(sm.getSecurityMarginBetween("sphere0", "sphere1"), 0.0)

    def test_overwrite_margin(self):
        """Setting margin twice should overwrite."""
        sm, *_ = create_security_margins_instance()

        sm.setSecurityMarginBetween("ur3", "sphere0", 0.05)
        sm.setSecurityMarginBetween("ur3", "sphere0", 0.10)

        self.assertEqual(sm.getSecurityMarginBetween("ur3", "sphere0"), 0.10)

    def test_apply_multiple_times(self):
        """Applying margins multiple times should not raise."""
        sm, _, graph, *_ = create_security_margins_instance()

        sm.defaultMargin = 0.02
        sm.apply()
        sm.defaultMargin = 0.03
        sm.apply()

        edges = graph.getTransitions()
        self.assertGreater(len(edges), 0)

    def test_large_margin_value(self):
        """Very large margin values should be accepted."""
        sm, *_ = create_security_margins_instance()

        sm.setSecurityMarginBetween("ur3", "sphere0", 100.0)

        self.assertEqual(sm.getSecurityMarginBetween("ur3", "sphere0"), 100.0)


if __name__ == "__main__":
    unittest.main()
