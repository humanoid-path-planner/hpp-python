#!/usr/bin/env python3
#
# Copyright (c) 2026 CNRS
# Author: Paul Sardin
#

import copy
import unittest
from pathlib import Path

import tomllib

from pyhpp.problem_file import load, load_file

EXAMPLE = Path(__file__).parents[2] / "doc" / "problem_file_example.toml"
UR5_URDF = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
UR5_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"


def example():
    return tomllib.loads(EXAMPLE.read_text())


class TestExample(unittest.TestCase):
    def test_builds_the_problem(self):
        p = load_file(EXAMPLE)
        self.assertEqual(p.name, "ur3-sphere")
        self.assertIn("free", p.graph.getStateNames())
        self.assertEqual(p.init.size, p.robot.model().nq)
        self.assertEqual(p.goal.size, p.robot.model().nq)
        self.assertEqual(p.planner.maxIterations(), 5000)

    def test_solves_the_problem(self):
        path = load_file(EXAMPLE).planner.solve()
        self.assertGreater(path.length(), 0)


class TestWithoutGraph(unittest.TestCase):
    def test_frame_constraint(self):
        p = load(
            {
                "format": "hpp-problem/0.1",
                "name": "ur5",
                "robots": [
                    {
                        "name": "ur5",
                        "urdf": UR5_URDF,
                        "srdf": UR5_SRDF,
                        "root_joint": "anchor",
                    }
                ],
                "constraints": {
                    "position": {
                        "type": "position",
                        "joint2": "ur5/wrist_3_joint",
                        "mask": [1, 1, 1],
                    }
                },
            }
        )
        self.assertIsNone(p.graph)
        self.assertIn("position", p.constraints)


class TestErrors(unittest.TestCase):
    def check(self, change, message):
        spec = copy.deepcopy(example())
        change(spec)
        with self.assertRaisesRegex(ValueError, message):
            load(spec)

    def test_format(self):
        self.check(lambda s: s.update(format="other/1.0"), "expected format")

    def test_unknown_key(self):
        self.check(lambda s: s.update(typo=0), "unknown keys in problem file")

    def test_unknown_nested_key(self):
        self.check(
            lambda s: s["problem"]["path_validation"].update(tollerance=0.5),
            "unknown keys in path validation",
        )

    def test_unknown_joint_in_constraint(self):
        self.check(
            lambda s: s["constraints"]["place_sphere"].update(joint2="sphere/typo"),
            "unknown joint",
        )

    def test_unknown_joint_in_configuration(self):
        self.check(lambda s: s["init"]["joints"].update(typo=0), "unknown joint")


if __name__ == "__main__":
    unittest.main()
