#!/usr/bin/env python

from argparse import ArgumentParser
from math import sqrt
import numpy as np
import datetime as dt

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
from pyhpp.manipulation import Device, Graph, Problem, urdf, ManipulationPlanner

from pyhpp.constraints import (
    LockedJoint,
)
from pinocchio import SE3, Quaternion

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

# Robot and object file paths
baxter_urdf = "package://example-robot-data/robots/baxter_description/urdf/baxter.urdf"
baxter_srdf = "package://example-robot-data/robots/baxter_description/srdf/baxter_manipulation.srdf"
table_urdf = "package://hpp_tutorial/urdf/table.urdf"
table_srdf = "package://hpp_tutorial/srdf/table.srdf"
box_urdf = "package://hpp_environments/urdf/baxter_benchmark/box.urdf"
box_srdf = "package://hpp_environments/srdf/baxter_benchmark/box.srdf"

# nbBoxes
K = 2
nBoxPerLine = 2
grippers = ["baxter/r_gripper", "baxter/l_gripper"]
# Box i will be at box goal[i] place at the end
goal = [1, 0]

robot = Device("baxter-manip")

# Load Baxter robot
baxter_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0.926]))
urdf.loadModel(robot, 0, "baxter", "anchor", baxter_urdf, baxter_srdf, baxter_pose)

# Load table
table_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "table", "anchor", table_urdf, table_srdf, table_pose)

# Load boxes
boxes = list()
for i in range(K):
    boxes.append("box" + str(i))
    box_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(
        robot,
        0,
        boxes[i],
        "freeflyer",
        box_urdf,
        box_srdf,
        box_pose,
    )
    robot.setJointBounds(
        boxes[i] + '/root_joint',
        [-1, 0.5, -1, 2, 0.6, 1.9, -1, 1, -1, 1, -1, 1, -1, 1]
    )

model = robot.model()

problem = Problem(robot)
cg = Graph("graph", robot, problem)

# Set error threshold and max iterations
cg.errorThreshold(1e-3)
cg.maxIterations(40)

q_init = robot.currentConfiguration()

# Calculate box positions
rankB = list()
for i in range(K):
    joint_id = robot.model().getJointId(boxes[i] + '/root_joint')
    rankB.append (robot.model().idx_qs[joint_id])

bb = [0.7, 0.8, 0., 0.1]
c = sqrt(2) / 2
xstep = (bb[1] - bb[0]) / (nBoxPerLine - 1) if nBoxPerLine > 1 else (bb[1] - bb[0])
nbCols = int(K * 1. / nBoxPerLine + 0.5)
ystep = (bb[3] - bb[2]) / (nbCols - 1) if nbCols > 1 else (bb[3] - bb[2])
for i in range(K):
    iL = i % nBoxPerLine
    iC = (i - iL) / nBoxPerLine
    x = bb[0] + xstep * iL
    y = bb[2] + xstep * iC
    q_init[rankB[i]:rankB[i]+7] = [x, y, 0.746, 0, -c, 0, c]

q_goal = q_init[::].copy()
for i in range(K):
    r = rankB[i]
    rn = rankB[goal[i]]
    q_goal[r:r+7] = q_init[rn:rn+7]

constraints = dict()
graphConstraints = dict()

jointNames = dict()
jointNames['all'] = robot.model().names
jointNames['baxterRightSide'] = list()
jointNames['baxterLeftSide'] = list()

for n in jointNames['all']:
    if n.startswith("baxter"):
        if n.startswith("baxter/left_"):
            jointNames['baxterLeftSide'].append(n)
        if n.startswith("baxter/right_"):
            jointNames['baxterRightSide'].append(n)
# Lock finger joints
lockFingers = ["r_gripper_l_finger",
               "r_gripper_r_finger",
               "l_gripper_l_finger",
               "l_gripper_r_finger",
        ]
for side in ["r", "l"]:
    joint_name = "baxter/" + side + "_gripper_r_finger_joint"
    cs = LockedJoint.create(robot.asPinDevice(), joint_name, np.array([-0.02]))
    constraints[side + "_gripper_r_finger"] = cs
    graphConstraints[side + "_gripper_r_finger"] = cs
    joint_name = "baxter/" + side + "_gripper_l_finger_joint"
    cs = LockedJoint.create(robot.asPinDevice(), joint_name, np.array([0.02]))
    constraints[side + "_gripper_l_finger"] = cs
    graphConstraints[side + "_gripper_l_finger"] = cs


# Lock head
lockHead = 'head_pan'
joint_name = 'baxter/head_pan'
joint_id = robot.model().getJointId(joint_name)
cs = LockedJoint.create(robot.asPinDevice(), joint_name, np.array([q_init[robot.model().idx_qs[joint_id]]]))
constraints[lockHead] = cs
graphConstraints[lockHead] = cs
for n in jointNames["baxterRightSide"]:
    cs = LockedJoint.create(robot.asPinDevice(), n, np.array([0.]))
    constraints[n] = cs

for n in jointNames["baxterLeftSide"]:
    cs = LockedJoint.create(robot.asPinDevice(), n, np.array([0.]))
    constraints[n] = cs

# Define handles and contact surfaces
handlesPerObject = list()
handles = list()
objContactSurfaces = list()
for i in range(K):
    handlesPerObject.append([boxes[i] + "/handle2"])
    handles.append(boxes[i] + "/handle2")
    objContactSurfaces.append([boxes[i] + "/box_surface"])

rules = [Rule([".*"], [".*"], True)]

factory = ConstraintGraphFactory(cg, constraints)
factory.setGrippers(grippers)
factory.environmentContacts(['table/pancake_table_table_top'])
factory.setObjects(boxes, handlesPerObject, objContactSurfaces)
factory.setRules(rules)
factory.generate()
cg.addNumericalConstraintsToGraph( list(graphConstraints.values()))
cg.initialize()

res = cg.applyStateConstraints(cg.getState('free'), q_init)
if not res.success:
    raise Exception('Init configuration could not be projected.')
q_init_proj = res.configuration

res = cg.applyStateConstraints(cg.getState('free'), q_goal)
if not res.success:
    raise Exception('Goal configuration could not be projected.')
q_goal_proj = res.configuration

problem.initConfig(q_init_proj)
problem.addGoalConfig(q_goal_proj)
problem.constraintGraph(cg)

manipulationPlanner = ManipulationPlanner(problem)

problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

optimizers = ['Graph-PartialShortcut', 'Graph-RandomShortcut', 'PartialShortcut', 
              'RandomShortcut', 'SimpleShortcut']
iOpt = 0

# Run benchmark
totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0
for i in range(args.N):
    # TODO: Clear and add path optimizers
    # ps.clearPathOptimizers()
    # ps.addPathOptimizer(optimizers[iOpt])
    # iOpt += 1
    # if iOpt == len(optimizers): iOpt = 0
    
    try:
        manipulationPlanner.roadmap().clear()
        problem.resetGoalConfigs()
        t1 = dt.datetime.now()
        manipulationPlanner.solve()
        t2 = dt.datetime.now()
    except Exception as e:
        print(f"Failed to plan path: {e}")
    else:
        success += 1
        totalTime += t2 - t1
        print(t2 - t1)
        n = len(manipulationPlanner.roadmap().nodes())
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

