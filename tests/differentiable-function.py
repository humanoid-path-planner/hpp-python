from pyhpp.pinocchio import LiegroupElement

from pyhpp.constraints import DifferentiableFunction
import numpy as np


class Function(DifferentiableFunction):
    def __init__(self):
        super(Function, self).__init__(2, 2, 2, "Test")

    def impl_compute(self, res, arg):
        res.v = 2 * arg

    def impl_jacobian(self, res, arg):
        res = 2 * np.eye(2)
        return res


function = Function()
print(function)

q = np.ones((function.ni, 1))

# C++ API
v = LiegroupElement(function.outputSpace())
function.value(v, q)
print(v)

J = np.zeros((function.ndo, function.ndi))
function.jacobian(J, q)
print(J)

# Pythonic API
v = function(q)
J = function.J(q)
