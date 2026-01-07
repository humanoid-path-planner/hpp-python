#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import datetime as dt

from pyhpp.core import Problem, DiffusingPlanner, Dichotomy, Straight
from pyhpp.pinocchio import Device, urdf
from pinocchio import SE3

parser = ArgumentParser()
parser.add_argument("-N", default=0, type=int)
parser.add_argument("--display", action="store_true")
parser.add_argument("--run", action="store_true")
args = parser.parse_args()

pr2_urdf = "package://example-robot-data/robots/pr2_description/urdf/pr2.urdf"
pr2_srdf = "package://example-robot-data/robots/pr2_description/srdf/pr2.srdf"
kitchen_urdf = "package://hpp_tutorial/urdf/kitchen_area_obstacle.urdf"

robot = Device("pr2")

pr2_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "pr2", "planar", pr2_urdf, pr2_srdf, pr2_pose)

robot.setJointBounds("pr2/root_joint", [-4, -3, -5, -3, -2, 2, -2, 2])

kitchen_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "kitchen", "anchor", kitchen_urdf, "", kitchen_pose)

model = robot.model()

q_init = robot.currentConfiguration()
q_goal = q_init.copy()

q_init[0:2] = [-3.2, -4]
rank = model.idx_qs[model.getJointId("pr2/torso_lift_joint")]
q_init[rank] = 0.2

q_goal[0:2] = [-3.2, -4]
rank = model.idx_qs[model.getJointId("pr2/l_shoulder_lift_joint")]
q_goal[rank] = 0.5
rank = model.idx_qs[model.getJointId("pr2/r_shoulder_lift_joint")]
q_goal[rank] = 0.5
rank = model.idx_qs[model.getJointId("pr2/l_elbow_flex_joint")]
q_goal[rank] = -0.5
rank = model.idx_qs[model.getJointId("pr2/r_elbow_flex_joint")]
q_goal[rank] = -0.5

problem = Problem(robot)
problem.pathValidation = Dichotomy(robot, 0.0)
problem.steeringMethod = Straight(problem)

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
planner = DiffusingPlanner(problem)
planner.maxIterations(5000)
totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0

for i in range(args.N):
    try:
        planner.roadmap().clear()
        problem.resetGoalConfigs()
        problem.initConfig(q_init)
        problem.addGoalConfig(q_goal)
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
