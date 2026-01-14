#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from pyhpp.pinocchio import LiegroupSpace, LiegroupElement


class TestLiegroupSpace(unittest.TestCase):

    def test_create_r1(self):
        space = LiegroupSpace.R1(False)

        self.assertIsNotNone(space)
        self.assertEqual(str(space), "R^1")

    def test_create_r2(self):
        space = LiegroupSpace.R2()

        self.assertEqual(str(space), "R^2")

    def test_create_r3(self):
        space = LiegroupSpace.R3()

        self.assertEqual(str(space), "R^3")

    def test_multiply_spaces(self):
        space1 = LiegroupSpace.R2()
        space2 = LiegroupSpace.R1(False)

        combined = space1 * space2

        self.assertEqual(str(combined), "R^3")

    def test_inplace_multiply(self):
        space = LiegroupSpace.R2()
        space *= LiegroupSpace.R1(False)

        self.assertEqual(str(space), "R^3")

    def test_merge_vector_spaces(self):
        space = LiegroupSpace.R2() * LiegroupSpace.R2()
        space.mergeVectorSpaces()

        self.assertEqual(str(space), "R^4")


class TestLiegroupElement(unittest.TestCase):

    def test_create_element(self):
        space = LiegroupSpace.R3()
        v = np.array([1.0, 2.0, 3.0])

        el = LiegroupElement(v, space)

        self.assertIsNotNone(el)

    def test_element_vector(self):
        space = LiegroupSpace.R3()
        v = np.array([1.0, 2.0, 3.0])

        el = LiegroupElement(v, space)

        np.testing.assert_array_equal(el.vector(), v)

    def test_element_space(self):
        space = LiegroupSpace.R3()
        v = np.array([1.0, 2.0, 3.0])

        el = LiegroupElement(v, space)

        self.assertEqual(el.space(), space)

    def test_element_subtraction(self):
        space = LiegroupSpace.R3()
        el1 = LiegroupElement(np.array([0.0, 1.0, 2.0]), space)
        el2 = LiegroupElement(np.array([1.0, 2.0, 3.0]), space)

        diff = el2 - el1

        np.testing.assert_array_almost_equal(diff.flatten(), [1.0, 1.0, 1.0])

    def test_element_addition(self):
        space = LiegroupSpace.R3()
        el1 = LiegroupElement(np.array([0.0, 1.0, 2.0]), space)
        v = np.array([1.0, 1.0, 1.0])

        el2 = el1 + v

        np.testing.assert_array_almost_equal(el2.vector(), [1.0, 2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
