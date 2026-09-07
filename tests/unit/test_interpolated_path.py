"""Interpolation points remain available when paths are copied or extracted."""

import numpy as np
from pyhpp.core import InterpolatedPath, interval
from pyhpp.core.path import Vector
from unit.conftest import create_ur5_problem


def test_interpolated_path_preserves_points_through_vector():
    _, robot = create_ur5_problem()
    q0 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
    q1 = q0.copy()
    q1[0] = 0.5
    path = InterpolatedPath(robot, q0, q0, interval(0.0, 2.0))
    path.insert(1.0, q1)
    vector = Vector(robot.configSize(), robot.numberDof())
    vector.appendPath(path)

    retrieved = vector.pathAtRank(0)
    assert isinstance(retrieved, InterpolatedPath)
    points = retrieved.interpolationPoints()
    assert [t for t, _ in points] == [0.0, 1.0, 2.0]
    np.testing.assert_allclose(points[1][1], q1)
    for start, end in ((0.0, 1.0), (1.0, 2.0)):
        piece = retrieved.extract(start, end)
        assert len(piece.interpolationPoints()) == 2
    points[1][1][0] = 99.0
    np.testing.assert_allclose(retrieved(1.0)[0], q1)
