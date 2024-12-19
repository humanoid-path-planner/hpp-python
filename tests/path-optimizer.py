from pyhpp.pinocchio import urdf
from pyhpp.core import ProblemSolver
import pyhpp.core.path
import numpy as np

from non_linear_spline_gradient_based import NonLinearSplineGradientBasedB3

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

ps.pathOptimizers.add(
    "NonLinearSplineGradientBasedB3", NonLinearSplineGradientBasedB3.create
)
ps.addPathOptimizer("NonLinearSplineGradientBasedB3")
ps.createPathOptimizers()
ps.clearPathOptimizers()

qinit = np.zeros((robot.model().nq, 1))
qgoal = qinit.copy()
qgoal[0, 0] = 1

ps.initConfig(qinit)
ps.addGoalConfig(qgoal)
ps.solve()

paths = ps.paths()
path = paths[0]

problem = ps.problem()
po = NonLinearSplineGradientBasedB3(problem)
optPath = po.optimize(path)

steer = problem.steeringMethod()
shooter = problem.configurationShooter()
for _ in range(10):
    qs = shooter.shoot()  # start
    qi = shooter.shoot()  # intermediate
    qe = shooter.shoot()  # end
    p1 = steer(qs, qi)
    p2 = steer(qi, qe)
    # valid = problem.pathValidation().validate (path, False, validPart, report)
    valid1, validPart, report = problem.pathValidation().validate(p1, False)
    valid2, validPart, report = problem.pathValidation().validate(p2, False)
    if not valid1 or not valid2:
        pathVector = pyhpp.core.path.Vector.create(
            robot.configSize(), robot.numberDof()
        )
        pathVector.appendPath(p1)
        pathVector.appendPath(p2)
        print(pathVector.numberPaths())
        po.optimize(pathVector)
