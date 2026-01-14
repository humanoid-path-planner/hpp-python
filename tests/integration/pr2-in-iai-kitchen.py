#!/usr/bin/env python

import numpy as np

from pyhpp.core import Problem, DiffusingPlanner, Dichotomy, Straight
from pyhpp.pinocchio import Device, urdf
from pinocchio import SE3

from benchmark_utils import create_benchmark_parser, run_benchmark_main

parser = create_benchmark_parser("PR2 Navigation in IAI Kitchen Benchmark")
args = parser.parse_args()

pr2_urdf = "package://example-robot-data/robots/pr2_description/urdf/pr2.urdf"
pr2_srdf = "package://example-robot-data/robots/pr2_description/srdf/pr2.srdf"
kitchen_urdf = "package://hpp_tutorial/urdf/kitchen_area_obstacle.urdf"

robot = Device('pr2')

pr2_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "pr2", "planar", pr2_urdf, pr2_srdf, pr2_pose)

robot.setJointBounds("pr2/root_joint", [-4, -3, -5, -3, -2, 2, -2, 2])

kitchen_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "kitchen", "anchor", kitchen_urdf, "", kitchen_pose)

model = robot.model()

q_init = robot.currentConfiguration()
q_goal = q_init.copy()

q_init[0:2] = [-3.2, -4]
rank = model.idx_qs[model.getJointId('pr2/torso_lift_joint')]
q_init[rank] = 0.2

q_goal[0:2] = [-3.2, -4]
rank = model.idx_qs[model.getJointId('pr2/l_shoulder_lift_joint')]
q_goal[rank] = 0.5
rank = model.idx_qs[model.getJointId('pr2/r_shoulder_lift_joint')]
q_goal[rank] = 0.5
rank = model.idx_qs[model.getJointId('pr2/l_elbow_flex_joint')]
q_goal[rank] = -0.5
rank = model.idx_qs[model.getJointId('pr2/r_elbow_flex_joint')]
q_goal[rank] = -0.5

problem = Problem(robot)
problem.pathValidation = Dichotomy(robot, 0.0)
problem.steeringMethod = Straight(problem)

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
planner = DiffusingPlanner(problem)
planner.maxIterations(5000)

# Run benchmark using shared utilities
if args.N > 0:
    run_benchmark_main(
        planner=planner,
        problem=problem,
        q_init=q_init,
        q_goal=q_goal,
        num_iterations=args.N,
    )
