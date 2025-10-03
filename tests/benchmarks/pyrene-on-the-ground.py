#!/usr/bin/env python

from argparse import ArgumentParser
from math import pi
import numpy as np
import datetime as dt

from pyhpp.core import Problem, DiffusingPlanner, createDichotomy, createProgressiveProjector, Straight
from pyhpp.constraints import LockedJoint, Implicit, Transformation, ComparisonType, ComparisonTypes
from pyhpp.pinocchio import Device, urdf
from pinocchio import SE3

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

# Robot file paths
talos_urdf = 'package://example-robot-data/robots/talos_data/robots/talos_full_v2.urdf'
talos_srdf = 'package://example-robot-data/robots/talos_data/srdf/talos.srdf'

robot = Device.create('pyrene')

# Load Talos robot
talos_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, 'pyrene', 'freeflyer', talos_urdf, talos_srdf, talos_pose)
robot.setJointBounds("pyrene/root_joint", [-3, 3, -3, 3, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1])

q0 = np.array([0, 0, 1.0192720229567027, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

problem = Problem(robot)

# Remove joint bound validation
problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

# TODO: Convert ps.addPartialCom - need to find pyhpp equivalent for partial COM constraints
# ps.addPartialCom ('pyrene', ['root_joint'])

# TODO: Convert robot.createSlidingStabilityConstraint - this is a humanoid-specific constraint
# The old implementation had:
# robot.createSlidingStabilityConstraint ("balance/", 'pyrene', robot.leftAnkle,
#                                         robot.rightAnkle, q0)
# Need to find how to create these constraints in pyhpp:
# - balance/relative-com
# - balance/relative-pose  
# - balance/pose-left-foot

leftAnkle = 'leg_left_6_joint'
rightAnkle = 'leg_right_6_joint'

constraints = dict()

# Lock gripper joints in closed position
model = robot.model()
jointNames = model.names

gripperJoints = [j for j in jointNames if j.startswith('gripper_')]
for j in gripperJoints:
    ljName = "locked_" + j
    joint_id = model.getJointId(j)
    value = q0[model.idx_qs[joint_id]]
    cs = LockedJoint.create(robot.asPinDevice(), j, np.array([value]))
    constraints[ljName] = cs

# Lock torso joints
for j in ['torso_1_joint', 'torso_2_joint']:
    ljName = "locked_" + j
    joint_id = model.getJointId(j)
    value = q0[model.idx_qs[joint_id]]
    cs = LockedJoint.create(robot.asPinDevice(), j, np.array([value]))
    constraints[ljName] = cs

# TODO: Add balance constraints to problem
# The old code used: ps.addNumericalConstraints ("balance", [...])
# Need to create and add these constraints once we know how to create them

# Set up path validation and projection
problem.pathValidation = createDichotomy(robot.asPinDevice(), 0)
problem.steeringMethod = Straight(problem)
problem.pathProjector = createProgressiveProjector(
    problem.distance(), problem.steeringMethod, 0.2
)

q1 = np.array([0.46186525119743765, 0.7691484390667176, 1.0, 0.044318662659833724, -0.0108631325758057, -0.0005624014939687202, 0.9989582234484302, 0.007182038754728065, -0.07804157609345573, -0.45119414082769843, 0.9175221606997885, -0.44402665063685365, -0.012200787668632173, 0.007200628661317587, -0.0788724231291963, -0.4956000845048934, 1.009916799073695, -0.49201388832345117, -0.011369733964913645, 0.0, 0.006761, 0.2408808670823526, 0.28558871367875255, 0.021347338765649856, -0.5979935578118766, -0.0014717027925961507, 0.006759032911476202, 0.08832103581416396, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0392843760345567, -0.10575191252250002, -0.05668798069441503, -1.7498341362736458, 0.0022744473854138473, 0.0015716871843022243, 0.07078184761729372, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00020052684970875304, 0.00019305414086825983])

q2 = np.array([-0.03792613133823603, 0.24583989035169654, 1.0, 0.008581421388221004, 0.044373915255123464, -0.0006050481369481731, 0.9989779520933587, 0.0011692308149178052, -0.011583002652064677, -0.5522315456639073, 0.9525259684676938, -0.4890594525896807, -0.007366718494771048, 0.0011679806161439602, -0.01159704912053673, -0.5610095845869443, 0.9704046648090222, -0.49816012449736746, -0.007352616506901346, 0.0, 0.006761, 0.25575424894162485, 0.21391256924828497, 0.006460912367916318, -0.5673886888192637, -0.0007964566272850148, 0.0027266557203091918, 0.09323792816834059, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5020992312243198, -0.770681876188843, 2.42600766027, -1.8794064100743089, 0.0019251455338804122, 0.007445905986286772, 0.06939811636044525, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0013061717130372818, 4.856617592856522e-05])

# TODO: Convert constraint application - need to find equivalent of ps.applyConstraints()
# The old code projected configurations onto constraints before validation
# May need to use problem or constraint graph methods
# Old: res, q1proj, err = ps.applyConstraints(q1)
# Possible new approach: apply constraints through problem or create a constraint set

# TODO: Convert config validation - need equivalent of robot.isConfigValid()
# Old: res, msg = robot.isConfigValid(q1proj)
# May need to use problem.configValidations or similar

# For now, assume configurations are valid
q1proj = q1
q2proj = q2

problem.initConfig(q1proj)
problem.addGoalConfig(q2proj)

planner = DiffusingPlanner(problem)
planner.maxIterations(5000)

# Run benchmark
totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0

for i in range(args.N):
    try:
        planner.roadmap().clear()
        problem.resetGoalConfigs()
        problem.initConfig(q1proj)
        problem.addGoalConfig(q2proj)
        t1 = dt.datetime.now()
        planner.solve()
        t2 = dt.datetime.now()
    except Exception as e:
        print(f"Failed to plan path: {e}")
    else:
        success += 1
        totalTime += t2 - t1
        print(t2 - t1)
        n = len(planner.roadmap().nodes())
        totalNumberNodes += n
        print("Number nodes: " + str(n))

if args.N != 0:
    print("#" * 20)
    print(f"Number of rounds: {args.N}")
    print(f"Number of successes: {success}")
    print(f"Success rate: {success / args.N * 100}%")
    if success > 0:
        print(f"Average time per success: {totalTime.total_seconds() / success}")
        print(f"Average number nodes per success: {totalNumberNodes / success}")