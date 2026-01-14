#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
from pyhpp.manipulation.constraint_graph_factory import (
    Constraints,
    PossibleGrasps,
    GraspIsAllowed,
    Rules,
    Rule,
)

class TestPossibleGrasps(unittest.TestCase):

    def test_allowed_grasp_returns_true(self):
        grippers = ["gripper1", "gripper2"]
        handles = ["handle1", "handle2", "handle3"]
        grasps = {
            "gripper1": ["handle1", "handle2"],
            "gripper2": ["handle3"]
        }
        validator = PossibleGrasps(grippers, handles, grasps)

        self.assertTrue(validator((0, 2)))  # gripper1->handle1, gripper2->handle3
        self.assertTrue(validator((1, 2)))  # gripper1->handle2, gripper2->handle3

    def test_forbidden_grasp_returns_false(self):
        grippers = ["gripper1", "gripper2"]
        handles = ["handle1", "handle2", "handle3"]
        grasps = {
            "gripper1": ["handle1"],
            "gripper2": ["handle3"]
        }
        validator = PossibleGrasps(grippers, handles, grasps)

        self.assertFalse(validator((1, 2)))  # gripper1->handle2 not allowed

    def test_none_grasp_allowed(self):
        grippers = ["gripper1"]
        handles = ["handle1"]
        grasps = {"gripper1": ["handle1"]}
        validator = PossibleGrasps(grippers, handles, grasps)

        self.assertTrue(validator((None,)))


class TestGraspIsAllowed(unittest.TestCase):

    def test_empty_validator_allows_all(self):
        validator = GraspIsAllowed()
        self.assertTrue(validator((0, 1, 2)))
        self.assertTrue(validator((None, None)))

    def test_appended_validator_is_called(self):
        validator = GraspIsAllowed()
        validator.append(lambda g: g[0] == 0)

        self.assertTrue(validator((0,)))
        self.assertFalse(validator((1,)))

    def test_all_validators_must_pass(self):
        validator = GraspIsAllowed()
        validator.append(lambda g: g[0] == 0)
        validator.append(lambda g: len(g) > 1)

        self.assertFalse(validator((0,)))  # fails second check
        self.assertTrue(validator((0, 1)))


class TestRules(unittest.TestCase):

    def test_simple_rule_allows(self):
        grippers = ["gripper1"]
        handles = ["handle1", "handle2"]
        rules = [Rule(grippers=["gripper1"], handles=["handle1"], link=True)]

        validator = Rules(grippers, handles, rules)

        self.assertTrue(validator((0,)))  # gripper1 -> handle1

    def test_simple_rule_forbids(self):
        grippers = ["gripper1"]
        handles = ["handle1", "handle2"]
        rules = [Rule(grippers=["gripper1"], handles=["handle1"], link=False)]

        validator = Rules(grippers, handles, rules)

        self.assertFalse(validator((0,)))  # gripper1 -> handle1 forbidden

    def test_regex_pattern_match(self):
        """Regex patterns in rules should match gripper/handle names."""
        grippers = ["left_gripper", "right_gripper"]
        handles = ["box_handle", "cup_handle"]
        # Rule: left_.* can grasp box_.* (regex matching)
        rules = [Rule(grippers=["left_.*"], handles=["box_.*"], link=True)]

        validator = Rules(grippers, handles, rules)

        # Tuple format: (handle_idx_for_gripper0, handle_idx_for_gripper1)
        # (0, None) = left_gripper grasps box_handle, right_gripper grasps nothing
        self.assertTrue(validator((0, None)))

if __name__ == "__main__":
    unittest.main()
