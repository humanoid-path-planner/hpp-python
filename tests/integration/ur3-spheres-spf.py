#!/usr/bin/env python3
#
#  Copyright 2020 CNRS
#
#  Author: Florent Lamiraux
#

from math import pi

import numpy as np
from benchmark_utils import create_benchmark_parser, run_benchmark_main
from pinocchio import SE3, Quaternion

from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    LockedJoint,
    Transformation,
)
from pyhpp.core import Dichotomy, Straight
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    ProgressiveProjector,
    StatesPathFinder,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory

parser = create_benchmark_parser("UR3 Spheres StatesPathFinder Benchmark")
args = parser.parse_args()

# Robot and environment file paths
ur3_urdf = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
ur3_srdf = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
sphere_urdf = "package://hpp_environments/urdf/construction_set/sphere.urdf"
sphere_srdf = "package://hpp_environments/srdf/construction_set/sphere.srdf"
ground_urdf = "package://hpp_environments/urdf/construction_set/ground.urdf"
ground_srdf = "package://hpp_environments/srdf/construction_set/ground.srdf"

nSphere = 2

robot = Device("ur3-spheres")

# Load UR3 robot
ur3_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "ur3", "anchor", ur3_urdf, ur3_srdf, ur3_pose)

# Change bounds of robots to increase workspace and avoid some collisions
robot.setJointBounds("ur3/shoulder_pan_joint", [-pi, 4])
robot.setJointBounds("ur3/shoulder_lift_joint", [-pi, 0])
robot.setJointBounds("ur3/elbow_joint", [-2.6, 2.6])

# Load ground
urdf.loadModel(
    robot,
    0,
    "ground",
    "anchor",
    ground_urdf,
    ground_srdf,
    SE3(rotation=np.identity(3), translation=np.array([0, 0, 0])),
)

# Load spheres to be manipulated
objects = []
sphere_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
for i in range(nSphere):
    urdf.loadModel(
        robot,
        0,
        f"sphere{i}",
        "freeflyer",
        sphere_urdf,
        sphere_srdf,
        sphere_pose,
    )
    robot.setJointBounds(
        f"sphere{i}/root_joint",
        [
            -1.0,
            1.0,
            -1.0,
            1.0,
            -0.1,
            1.0,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
        ],
    )
    objects.append(f"sphere{i}")

model = robot.model()

problem = Problem(robot)
cg = Graph("manipulation", robot, problem)

# Set error threshold and max iterations
cg.errorThreshold(1e-4)
cg.maxIterations(40)

constraints = {}

## Grippers
grippers = ["ur3/gripper"]

## Handles
handlesPerObject = [[f"sphere{i}/handle"] for i in range(nSphere)]
contactsPerObject = [[] for i in range(nSphere)]

## Constraints
for i in range(nSphere):
    o = objects[i]
    h = robot.handles()[o + "/handle"]
    h.mask = [True, True, True, False, True, True]

    # placement constraint
    placementName = f"place_sphere{i}"
    Id = SE3.Identity()
    q = Quaternion(1, 0, 0, 0)
    ballPlacement = SE3(q, np.array([0, 0, 0.02]))
    joint = robot.model().getJointId(f"sphere{i}/root_joint")
    pc = Transformation(
        placementName,
        robot,
        joint,
        Id,
        ballPlacement,
        [False, False, True, True, True, False],
    )
    cts = ComparisonTypes()
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    implicit_mask = [True, True, True]
    implicitPlacementConstraint = Implicit(pc, cts, implicit_mask)
    constraints[placementName] = implicitPlacementConstraint

    # placement complement constraint
    pc = Transformation(
        placementName + "/complement",
        robot,
        joint,
        Id,
        ballPlacement,
        [True, True, False, False, False, True],
    )
    cts[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    )
    implicit_mask = [True, True, True]
    implicitPlacementComplementConstraint = Implicit(pc, cts, implicit_mask)
    constraints[placementName + "/complement"] = implicitPlacementComplementConstraint

    # combination of placement and complement
    cts[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.Equality,
    )
    ll = LockedJoint(
        robot,
        f"sphere{i}/root_joint",
        np.array([0, 0, 0.02, 0, 0, 0, 1]),
        cts,
    )
    constraints[placementName + "/hold"] = ll
    cg.registerConstraints(
        constraints[placementName],
        constraints[placementName + "/complement"],
        constraints[placementName + "/hold"],
    )

    # preplacement constraint
    preplacementName = f"preplace_sphere{i}"
    Id = SE3.Identity()
    q = Quaternion(1, 0, 0, 0)
    ballPrePlacement = SE3(q, np.array([0, 0, 0.1]))
    joint = robot.model().getJointId(f"sphere{i}/root_joint")
    pc = Transformation(
        preplacementName,
        robot,
        joint,
        Id,
        ballPrePlacement,
        [False, False, True, True, True, False],
    )
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    implicit_mask = [True, True, True]
    implicitPrePlacementConstraint = Implicit(pc, cts, implicit_mask)
    constraints[preplacementName] = implicitPrePlacementConstraint

q_init = [
    pi / 6,
    -pi / 2,
    pi / 2,
    0,
    0,
    0,
    0.2,
    0,
    0.02,
    0,
    0,
    0,
    1,
    0.3,
    0,
    0.02,
    0,
    0,
    0,
    1,
]
q_goal = [
    pi / 6,
    -pi / 2,
    pi / 2,
    0,
    0,
    0,
    0.3,
    0,
    0.02,
    0,
    0,
    0,
    1,
    0.2,
    0,
    0.02,
    0,
    0,
    0,
    1,
]

factory = ConstraintGraphFactory(cg, constraints)

factory.setGrippers(grippers)
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.generate()

# Uncomment to help M-RRT pathplanner
# for e in ['ur3/gripper > sphere0/handle | f_ls',
#           'ur3/gripper > sphere1/handle | f_ls'] :
#  cg.setWeight(cg.getTransition(e), 100)
# for e in ['ur3/gripper < sphere0/handle | 0-0_ls',
#           'ur3/gripper < sphere1/handle | 0-1_ls'] :
#  cg.setWeight(cg.getTransition(e), 100)

for i in range(nSphere):
    e = cg.getTransition(f"ur3/gripper > sphere{i}/handle | f_23")
    cg.addNumericalConstraintsToTransition(
        e, [constraints[f"place_sphere{i}/complement"]]
    )
    e = cg.getTransition(f"ur3/gripper < sphere{i}/handle | 0-{i}_32")
    cg.addNumericalConstraintsToTransition(
        e, [constraints[f"place_sphere{i}/complement"]]
    )

problem.steeringMethod(Straight(problem))
problem.pathValidation(Dichotomy(robot, 0))
problem.pathValidationFactory(Dichotomy(robot, 0))

# need to set path projector due to implicit constraints added above
problem.pathProjector(
    ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.01)
)

cg.initialize()

q_init_arr = np.array(q_init)
q_goal_arr = np.array(q_goal)

problem.initConfig(q_init_arr)
problem.addGoalConfig(q_goal_arr)
problem.constraintGraph(cg)

planner = StatesPathFinder(problem)
planner.maxIterations(5000)

problem.setParameter("StatesPathFinder/innerPlannerTimeOut", 0.0)
problem.setParameter("StatesPathFinder/innerPlannerMaxIterations", 100)
problem.setParameter("StatesPathFinder/nTriesUntilBacktrack", 3)

# Run benchmark using shared utilities
if args.N > 0:
    run_benchmark_main(
        planner=planner,
        problem=problem,
        q_init=q_init_arr,
        q_goal=q_goal_arr,
        num_iterations=args.N,
    )
