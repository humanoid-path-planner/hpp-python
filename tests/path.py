import unittest

import numpy as np
import pyhpp.core


class PythonStraightPath(pyhpp.core.PathWrap):
    def __init__(self, a: float, b: float, v: float):
        duration = (b - a) / v
        super().__init__(pyhpp.core.interval(0, duration), 1, 1)
        self.a = a
        self.b = b
        self.v = v
        self.initPtr(self)
        self._counters = {}

    def _inc_counter(self, key):
        self._counters[key] = self._counters.get(key, 0) + 1

    def copy(self):
        self._inc_counter("copy")
        p = PythonStraightPath(self.a, self.b, self.v)
        return p

    def initial(self):
        self._inc_counter("initial")
        return self.a

    def end(self):
        self._inc_counter("end")
        return self.b

    def impl_compute(self, param):
        self._inc_counter("impl_compute")
        return (
            np.array(
                [
                    self.a + param * self.v,
                ]
            ),
            True,
        )

    def impl_derivative(self, param, order):
        self._inc_counter("impl_derivative")
        if order == 1:
            return np.array(
                [
                    self.v,
                ]
            )
        else:
            return np.array(
                [
                    0.0,
                ]
            )

    def clear_counter(self, key=None):
        if key is None:
            self._counters.clear()
        else:
            self._counters["key"] = 0

    def assert_called(self, key):
        assert self._counters.get(key, 0) > 0


class TestPath(unittest.TestCase):
    def test_path_api(self):
        space = pyhpp.pinocchio.LiegroupSpace.R1(False)
        path = pyhpp.core.StraightPath.create(
            space,
            np.array((0.0,)),
            np.array((1.0,)),
            pyhpp.core.interval(1.0, 2.0),
            None,
        )
        self.assertEqual(path.initial(), 0.0)
        self.assertEqual(path.end(), 1.0)
        self.assertEqual(path.length(), 1.0)

        self.assertEqual(path.eval(1.5), (0.5, True))
        self.assertEqual(path.derivative(1.5, 1), 1.0)
        self.assertEqual(path.derivative(1.5, 2), 0.0)

        extracted_path = path.extract(1.5, 2.0)
        self.assertTrue(isinstance(extracted_path, pyhpp.core.StraightPath))
        self.assertEqual(extracted_path.length(), 0.5)

    def test_path_inheritance(self):
        path = PythonStraightPath(2.0, 4.0, 1.0)

        # path.initial()
        res, ok = path.eval(1.0)
        self.assertTrue(ok)
        self.assertEqual(res, 3.0)
        path.assert_called("impl_compute")

        res = path.derivative(1.0, 1)
        self.assertEqual(res, 1.0)
        path.assert_called("impl_derivative")

        copy = path.copy()
        self.assertEqual(copy.initial(), path.initial())
        self.assertEqual(copy.end(), path.end())

        # Check that it can be used as a path
        pv = pyhpp.core.path.Vector.create(1, 1)
        pv.appendPath(path)

        path.clear_counter("copy")
        _path_0 = pv.pathAtRank(0)
        path.assert_called("copy")


if __name__ == "__main__":
    unittest.main()
