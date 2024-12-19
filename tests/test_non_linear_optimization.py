from pyhpp.core import ProblemSolver
from pyhpp.corbaserver import Server

from non_linear_spline_gradient_based import NonLinearSplineGradientBasedB3

ps = ProblemSolver.create()
ps.pathOptimizers.add(
    "NonLinearSplineGradientBasedB3", NonLinearSplineGradientBasedB3.create
)

server = Server(ps, False)
server.startCorbaServer()
server.processRequest(True)
del server
