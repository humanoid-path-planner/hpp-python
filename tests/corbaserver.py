from pyhpp.core import ProblemSolver
from pyhpp.corbaserver import Server

ps = ProblemSolver.create()

for i in range(3):
    print i
    server = Server (ps, False)
    server.startCorbaServer()
    server.processRequest (True)
    del server
