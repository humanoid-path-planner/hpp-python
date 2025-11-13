from math import pi
import numpy as np

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation import Device, urdf, Graph, Problem
from pyhpp.constraints import (
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
    LockedJoint,
)
from pinocchio import SE3, Quaternion

# based on /hpp_benchmark/2025/04-01/ur3-spheres/script.py

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
sphere_pose = SE3(rotation=np.identity(3), translation=np.array([-2.5, -4, 0.746]))
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

# Load kitchen environment
kitchen_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(
    robot, 0, "kitchen_area", "anchor", ground_urdf, ground_srdf, kitchen_pose
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
    q = Quaternion(0, 0, 0, 1)
    ballPlacement = SE3(q, np.array([0, 0, 0.02]))
    joint = robot.model().getJointId("sphere{0}/root_joint".format(i))
    pc = Transformation(
        placementName,
        robot,
        joint,
        ballPlacement,
        Id,
        [True, True, False, False, False, True],
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
        ballPlacement,
        Id,
        [False, False, True, True, True, False],
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
    q = Quaternion(0, 0, 0, 1)
    ballPrePlacement = SE3(q, np.array([0, 0, 0.1]))
    joint = robot.model().getJointId("sphere{0}/root_joint".format(i))
    pc = Transformation(
        preplacementName,
        robot,
        joint,
        ballPrePlacement,
        Id,
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

cg.maxIterations(100)
cg.errorThreshold(0.0001)
factory = ConstraintGraphFactory(cg, constraints)

factory.setGrippers(grippers)
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.generate()

cg.initialize()
# cg.display("./temp.dot")
# problem.initConfig(q_init)
# problem.addGoalConfig(q_goal)

print("Script completed!")
