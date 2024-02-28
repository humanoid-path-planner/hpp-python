from pyhpp.pinocchio import urdf
from pyhpp.core import ProblemSolver
import pyhpp.core.path
import pinocchio, eigenpy
import numpy as np

ps = ProblemSolver.create()
robot = ps.createRobot("ur3")
urdf.loadRobotModel(
    robot,
    "anchor",
    "example-robot-data/robots/ur_description",
    "ur3",
    "_gripper",
    "_gripper",
)
ps.robot(robot)

qinit = np.zeros((robot.model().nq, 1))
qgoal = qinit.copy()
qgoal[0, 0] = 1

ps.initConfig(qinit)
ps.addGoalConfig(qgoal)
ps.solve()

paths = ps.paths()
path = paths[0]

assert (path.initial() == qinit).all()

q, success = path(0.3)
assert success

qq = np.zeros_like(qinit)
success = path(qq, path.length())
assert success
