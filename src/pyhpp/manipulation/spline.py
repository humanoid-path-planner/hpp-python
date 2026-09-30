"""Spline smoothing of paths carrying manipulation graph constraints."""

from pyhpp.core import Straight
from pyhpp.core.path import Vector
from pyhpp.manipulation import SplineGradientBased_bezier3


class ManipulationSpline(SplineGradientBased_bezier3):
    """Smooth transitions, then join splines within each manipulation state.

    The result contains one PathVector per consecutive group of transitions
    with the same containing state. These groups can be timed independently
    to preserve stops at state changes.

    ``singleSplineTransitions`` selects transitions to fit from their endpoints
    using their existing constraints, for example constrained insertion motions.
    Other transitions retain their interpolation intervals as the initial spline
    pieces. Input paths must be geometric paths with correct transition labels.
    """

    def __init__(self, problem, graph):
        super().__init__(problem)
        self.graph = graph
        self.straight = Straight(problem)
        self.singleSplineTransitions = ()
        self.costOrder = 2
        self.maxIterations(100)

    def optimize(self, path):
        """Return smoothed state groups with their graph constraints preserved."""
        flat = Vector(path.outputSize(), path.outputDerivativeSize())
        path.flatten(flat)
        groups = []
        previous = None
        cursor = 0.0
        for rank in range(flat.numberPaths()):
            leaf = flat.pathAtRank(rank)
            start, cursor = cursor, cursor + leaf.length()
            if cursor - start <= 1e-9:
                continue
            name = self.graph.transitionAtParam(path, (start + cursor) / 2).name()
            if name != previous:
                groups.append(Vector(path.outputSize(), path.outputDerivativeSize()))
                previous = name
            groups[-1].appendPath(leaf)

        states = []
        previous = None
        for group in groups:
            if group.length() < 1e-6:
                raise ValueError("Path transition is too short for spline timing")
            transition = self.graph.transitionAtParam(group, group.length() / 2)
            if transition.name() in self.singleSplineTransitions:
                self.straight.constraints(group.pathAtRank(0).constraints())
                insertion = self.straight(group.initial(), group.end())
                group = Vector(path.outputSize(), path.outputDerivativeSize())
                group.appendPath(insertion)
            state = self.graph.getContainingNode(transition)
            if state != previous:
                states.append(Vector(path.outputSize(), path.outputDerivativeSize()))
                previous = state
            states[-1].concatenate(super().optimize(group))

        result = Vector(path.outputSize(), path.outputDerivativeSize())
        for group in states:
            result.appendPath(super().optimize(group))
        return result
