from pyhpp.core import ProblemSolver
from pyhpp.core.path_optimization import SplineGradientBasedAbstractB3
from pyhpp.corbaserver import Server
from pyhpp.corbaserver.wholebodyStep import Server as WBServer

from non_linear_spline_gradient_based import NonLinearSplineGradientBasedB3

ps = ProblemSolver.create()
ps.pathOptimizers.add ("NonLinearSplineGradientBasedB3", NonLinearSplineGradientBasedB3.create)

def serve (ps):
    server = Server (ps, False)
    wbserver = WBServer (server, False)
    server.startCorbaServer()
    wbserver.startCorbaServer (server)
    server.processRequest (True)
    del server
    del wbserver

serve(ps)
ps.optimizePath(ps.paths()[0])
