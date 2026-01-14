#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem


class TestConfigurationShooter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_returns_valid_configuration(self):
        shooter = self.problem.configurationShooter()

        q = shooter.shoot()

        self.assertEqual(len(q), self.robot.configSize())


class TestConfigurationShooterEdgeCases(unittest.TestCase):
    """Edge case tests for ConfigurationShooter."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()

    def test_shot_respects_joint_bounds(self):
        """Shot configurations should respect joint bounds."""
        shooter = self.problem.configurationShooter()

        for _ in range(10):
            q = shooter.shoot()
            for i in range(len(q)):
                self.assertFalse(
                    np.isnan(q[i]),
                    f"Configuration element {i} is NaN"
                )
                self.assertFalse(
                    np.isinf(q[i]),
                    f"Configuration element {i} is infinite"
                )


if __name__ == "__main__":
    unittest.main()
