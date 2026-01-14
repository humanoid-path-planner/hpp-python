#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pyhpp.pinocchio import LiegroupElement
from pyhpp.constraints import DifferentiableFunction


class DoubleFunction(DifferentiableFunction):
    """Simple function that doubles the input."""

    def __init__(self):
        super().__init__(2, 2, 2, "Double")

    def impl_compute(self, res, arg):
        res.v = 2 * arg

    def impl_jacobian(self, res, arg):
        res = 2 * np.eye(2)
        return res


class TestDifferentiableFunctionInheritance(unittest.TestCase):

    def test_create_subclass(self):
        func = DoubleFunction()

        self.assertIsNotNone(func)

    def test_function_name(self):
        func = DoubleFunction()

        self.assertIn("Double", str(func))

    def test_input_output_sizes(self):
        func = DoubleFunction()

        self.assertEqual(func.ni, 2)
        self.assertEqual(func.ndo, 2)
        self.assertEqual(func.ndi, 2)

    def test_value(self):
        func = DoubleFunction()
        q = np.ones((func.ni, 1))

        v = LiegroupElement(func.outputSpace())
        func.value(v, q)

        np.testing.assert_array_almost_equal(v.vector(), [2.0, 2.0])

    def test_jacobian(self):
        func = DoubleFunction()
        q = np.ones((func.ni, 1))

        J = np.zeros((func.ndo, func.ndi))
        func.jacobian(J, q)

        expected = 2 * np.eye(2)
        np.testing.assert_array_almost_equal(J, expected)

    def test_call_operator(self):
        func = DoubleFunction()
        q = np.ones((func.ni, 1))

        v = func(q)

        self.assertIsNotNone(v)

    def test_jacobian_shorthand(self):
        func = DoubleFunction()
        q = np.ones((func.ni, 1))

        J = func.J(q)

        self.assertEqual(J.shape, (func.ndo, func.ndi))


if __name__ == "__main__":
    unittest.main()
