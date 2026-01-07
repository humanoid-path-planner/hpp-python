#!/usr/bin/env python

from argparse import ArgumentParser
from math import sqrt
import numpy as np
import datetime as dt

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
from pyhpp.manipulation import Device, Graph, Problem, urdf, ManipulationPlanner
from pyhpp.core import Dichotomy, Straight, ProgressiveProjector
from pyhpp.constraints import LockedJoint
from pinocchio import SE3

parser = ArgumentParser()
parser.add_argument('-N', default=0, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

pr2_urdf = "package://example-robot-data/robots/pr2_description/urdf/pr2.urdf"
pr2_srdf = "package://example-robot-data/robots/pr2_description/srdf/pr2_manipulation.srdf"
box_urdf = "package://hpp_tutorial/urdf/box.urdf"
box_srdf = "package://hpp_tutorial/srdf/box.srdf"
kitchen_urdf = "package://hpp_tutorial/urdf/kitchen_area_obstacle.urdf"
kitchen_srdf = "package://hpp_tutorial/srdf/kitchen_area.srdf"

robot = Device('pr2-box')

pr2_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "pr2", "planar", pr2_urdf, pr2_srdf, pr2_pose)

box_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "box", "freeflyer", box_urdf, box_srdf, box_pose)

kitchen_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "kitchen_area", "anchor", kitchen_urdf, kitchen_srdf, kitchen_pose)

robot.setJointBounds("pr2/root_joint", [-5, -2, -5.2, -2.7])
robot.setJointBounds("box/root_joint", [-5.1, -2, -5.2, -2.7, 0, 1.5])

model = robot.model()

problem = Problem(robot)
problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

cg = Graph('graph', robot, problem)
cg.errorThreshold(1e-3)
cg.maxIterations(40)

constraints = dict()

c = sqrt(2) / 2

q_init = robot.currentConfiguration()
q_init[0:4] = [-3.2, -4, 1, 0]

rank = robot.rankInConfiguration['pr2/r_gripper_l_finger_joint']
q_init[rank] = 0.5
rank = robot.rankInConfiguration['pr2/r_gripper_r_finger_joint']
q_init[rank] = 0.5
rank = robot.rankInConfiguration['pr2/l_gripper_l_finger_joint']
q_init[rank] = 0.5
rank = robot.rankInConfiguration['pr2/l_gripper_r_finger_joint']
q_init[rank] = 0.5
rank = robot.rankInConfiguration['pr2/torso_lift_joint']
q_init[rank] = 0.2

q_goal = q_init.copy()

rank = robot.rankInConfiguration['box/root_joint']
q_init[rank:rank+7] = [-2.5, -3.6, 0.76, 0, c, 0, c]
rank = robot.rankInConfiguration['box/root_joint']
q_goal[rank:rank+7] = [-2.5, -4.4, 0.76, 0, -c, 0, c]

locklhand = ['l_l_finger', 'l_r_finger']
cs = LockedJoint(robot, 'pr2/l_gripper_l_finger_joint', np.array([0.5]))
constraints['l_l_finger'] = cs
cs = LockedJoint(robot, 'pr2/l_gripper_r_finger_joint', np.array([0.5]))
constraints['l_r_finger'] = cs

lockrhand = ['r_l_finger', 'r_r_finger']
cs = LockedJoint(robot, 'pr2/r_gripper_l_finger_joint', np.array([0.5]))
constraints['r_l_finger'] = cs
cs = LockedJoint(robot, 'pr2/r_gripper_r_finger_joint', np.array([0.5]))
constraints['r_r_finger'] = cs

lockhands = lockrhand + locklhand

lockHeadAndTorso = ['head_pan', 'head_tilt', 'torso', 'laser']
cs = LockedJoint(robot, 'pr2/head_pan_joint', 
                 np.array([q_init[robot.rankInConfiguration['pr2/head_pan_joint']]]))
constraints['head_pan'] = cs
cs = LockedJoint(robot, 'pr2/head_tilt_joint',
                 np.array([q_init[robot.rankInConfiguration['pr2/head_tilt_joint']]]))
constraints['head_tilt'] = cs
cs = LockedJoint(robot, 'pr2/torso_lift_joint',
                 np.array([q_init[robot.rankInConfiguration['pr2/torso_lift_joint']]]))
constraints['torso'] = cs
cs = LockedJoint(robot, 'pr2/laser_tilt_mount_joint',
                 np.array([q_init[robot.rankInConfiguration['pr2/laser_tilt_mount_joint']]]))
constraints['laser'] = cs

lockPlanar = ['lockplanar']
rank = robot.rankInConfiguration['pr2/root_joint']
cs = LockedJoint(robot, 'pr2/root_joint', np.array([-3.2, -4, 1, 0]))
constraints['lockplanar'] = cs

lockAll = lockhands + lockHeadAndTorso + lockPlanar

grippers = ['pr2/l_gripper', 'pr2/r_gripper']
boxes = ['box']
handlesPerObject = [['box/handle', 'box/handle2']]
objContactSurfaces = [['box/box_surface']]
envSurfaces = ['kitchen_area/pancake_table_table_top']

rules = [Rule([""], [""], True)]

factory = ConstraintGraphFactory(cg, constraints)
factory.setGrippers(grippers)
factory.environmentContacts(envSurfaces)
factory.setObjects(boxes, handlesPerObject, objContactSurfaces)
factory.setRules(rules)
factory.generate()

cg.addNumericalConstraintsToGraph([constraints[c] for c in lockAll])

problem.steeringMethod = Straight(problem)
problem.pathValidation = Dichotomy(robot, 0.0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod, 0.2
)

cg.initialize()

res, q_init_proj, err = cg.applyStateConstraints(cg.getState('free'), q_init)
if not res:
    raise RuntimeError("Failed to project initial configuration")

res, q_goal_proj, err = cg.applyStateConstraints(cg.getState('free'), q_goal)
if not res:
    raise RuntimeError("Failed to project goal configuration")

problem.initConfig(q_init_proj)
problem.addGoalConfig(q_goal_proj)
problem.constraintGraph(cg)

manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(5000)

totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0

for i in range(args.N):
    try:
        manipulationPlanner.roadmap().clear()
        problem.resetGoalConfigs()
        problem.initConfig(q_init_proj)
        problem.addGoalConfig(q_goal_proj)
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