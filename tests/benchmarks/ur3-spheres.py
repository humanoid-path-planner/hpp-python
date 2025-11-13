from math import pi
import numpy as np
import datetime as dt

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    ProgressiveProjector,
    urdf,
    ManipulationPlanner,
)
from pyhpp.core import Dichotomy, Straight

from pyhpp.constraints import (
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
    LockedJoint,
)
from pinocchio import SE3, Quaternion

# based on /hpp_benchmark/2025/04-01/ur3-spheres/script.py
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-N", default=20, type=int)
args = parser.parse_args()
# Robot and environment file paths
ur3_urdf = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
ur3_srdf = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
sphere_urdf = "package://hpp_environments/urdf/construction_set/sphere.urdf"
sphere_srdf = "package://hpp_environments/srdf/construction_set/sphere.srdf"
ground_urdf = "package://hpp_environments/urdf/construction_set/ground.urdf"
ground_srdf = "package://hpp_environments/srdf/construction_set/ground.srdf"

robot = Device("ur3-spheres")

# Load UR3 robot
ur3_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "ur3", "anchor", ur3_urdf, ur3_srdf, ur3_pose)

# Load sphere to be manipulated
objects = list()
nSphere = 2
sphere_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
for i in range(nSphere):
    urdf.loadModel(
        robot,
        0,
        "sphere{0}".format(i),
        "freeflyer",
        sphere_urdf,
        sphere_srdf,
        sphere_pose,
    )
    robot.setJointBounds(
        "sphere{0}/root_joint".format(i),
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
    objects.append("sphere{0}".format(i))

urdf.loadModel(
    robot,
    0,
    "kitchen_area",
    "anchor",
    ground_urdf,
    ground_srdf,
    SE3(rotation=np.identity(3), translation=np.array([0, 0, 0])),
)

model = robot.model()
robot.setJointBounds("ur3/shoulder_pan_joint", [-pi, 4])
robot.setJointBounds("ur3/shoulder_lift_joint", [-pi, 0])
robot.setJointBounds("ur3/elbow_joint", [-2.6, 2.6])

problem = Problem(robot)
cg = Graph("graph", robot, problem)

constraints = dict()

for i in range(nSphere):
    o = objects[i]
    h = robot.handles()[o + "/handle"]
    h.mask = [True, True, True, False, True, True]
    # placement constraint
    placementName = "place_sphere{0}".format(i)
    Id = SE3.Identity()
    q = Quaternion(1, 0, 0, 0)
    ballPlacement = SE3(q, np.array([0, 0, 0.02]))
    joint = robot.model().getJointId("sphere{0}/root_joint".format(i))
    pc = Transformation(
        placementName,
        robot.asPinDevice(),
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
        robot.asPinDevice(),
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
    ll = LockedJoint.createWithComp(
        robot.asPinDevice(),
        "sphere{0}/root_joint".format(i),
        np.array([0, 0, 0.02, 0, 0, 0, 1]),
        cts,
    )
    constraints[placementName + "/hold"] = ll
    cg.registerConstraints(
        constraints[placementName],
        constraints[placementName + "/complement"],
        constraints[placementName + "/hold"],
    )

    preplacementName = "preplace_sphere{0}".format(i)
    Id = SE3.Identity()
    q = Quaternion(1, 0, 0, 0)
    ballPrePlacement = SE3(q, np.array([0, 0, 0.1]))
    joint = robot.model().getJointId("sphere{0}/root_joint".format(i))
    pc = Transformation(
        preplacementName,
        robot.asPinDevice(),
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

grippers = ["ur3/gripper"]
handlesPerObject = [["sphere{0}/handle".format(i)] for i in range(nSphere)]
contactsPerObject = [[] for i in range(nSphere)]

cg.maxIterations(40)
cg.errorThreshold(0.0001)
factory = ConstraintGraphFactory(cg, constraints)

factory.setGrippers(grippers)
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.generate()

# Uncomment to help M-RRT pathplanner
# for e in ['ur3/gripper > sphere0/handle | f_ls',
#           'ur3/gripper > sphere1/handle | f_ls'] :
#  cg.setWeight(cg.getTransition(e), 100)
# for e in ['ur3/gripper < sphere0/handle | 0-0_ls',
#          'ur3/gripper < sphere1/handle | 0-1_ls'] :
#  cg.setWeight(cg.getTransition(e), 100)

for i in range(nSphere):
    e = cg.getTransition("ur3/gripper > sphere{}/handle | f_23".format(i))
    cg.addNumericalConstraintsToTransition(
        e, [constraints["place_sphere{}/complement".format(i)]]
    )
    e = cg.getTransition("ur3/gripper < sphere{}/handle | 0-{}_32".format(i, i))
    cg.addNumericalConstraintsToTransition(
        e, [constraints["place_sphere{}/complement".format(i)]]
    )

problem.steeringMethod = Straight(problem)
problem.pathValidation = Dichotomy(robot.asPinDevice(), 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod, 0.01
)

cg.initialize()

problem.initConfig(np.array(q_init))
problem.addGoalConfig(np.array(q_goal))
problem.constraintGraph(cg)
manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(5000)

# Run benchmark
#

totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0
for i in range(args.N):
    try:
        manipulationPlanner.roadmap().clear()
        t1 = dt.datetime.now()
        manipulationPlanner.solve()
        t2 = dt.datetime.now()
    except Exception as e:
        print(f"Failed to plan path: {e}")
        break
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
