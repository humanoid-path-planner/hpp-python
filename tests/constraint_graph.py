from pyhpp.manipulation import Device, urdf, Graph, Problem
from pyhpp.core import ConfigurationShooter  # noqa: F401
import numpy as np
from pinocchio import SE3, StdVec_Bool as Mask, Quaternion


from pyhpp.constraints import (
    RelativeTransformation,
    Transformation,
    ComparisonTypes,
    ComparisonType,
    BySubstitution,
    Implicit,
)

urdfFilename = (
    "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
)
srdfFilename = (
    "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"
)

urdfFilenameBall = "package://hpp_environments/urdf/ur_benchmark/pokeball.urdf"
srdfFilenameBall = "package://hpp_environments/srdf/ur_benchmark/pokeball.srdf"

r0_pose = SE3(rotation=np.identity(3), translation=np.array([-0.25, 0, 0]))
r1_pose = SE3(rotation=np.identity(3), translation=np.array([1, 0, 0]))

robot = Device("bot")

urdf.loadModel(robot, 0, "ur5", "anchor", urdfFilename, srdfFilename, r0_pose)
urdf.loadModel(
    robot, 0, "pokeball", "freeflyer", urdfFilenameBall, srdfFilenameBall, r1_pose
)


robot.setJointBounds(
    "pokeball/root_joint",
    [
        -0.4,
        0.4,
        -0.4,
        0.4,
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

problem = Problem(robot)

graph = Graph("graph", robot, problem)

# Create states
state_placement = graph.createState("placement", False, 0)
state_grasp_placement = graph.createState("grasp-placement", False, 0)
state_gripper_above_ball = graph.createState("gripper-above-ball", False, 0)
state_ball_above_ground = graph.createState("ball-above-ground", False, 0)
state_grasp = graph.createState("grasp", False, 0)

# Create transitions
transitions_transit = graph.createTransition(
    state_placement, state_placement, "transit", 1, state_placement
)

transitions_approach_ball = graph.createTransition(
    state_placement, state_gripper_above_ball, "approach-ball", 1, state_placement
)
transitions_move_gripper_away = graph.createTransition(
    state_gripper_above_ball, state_placement, "move-gripper-away", 1, state_placement
)
transitions_grasp_ball = graph.createTransition(
    state_gripper_above_ball, state_grasp_placement, "grasp-ball", 1, state_placement
)
transitions_move_gripper_up = graph.createTransition(
    state_grasp_placement,
    state_gripper_above_ball,
    "move-gripper-up",
    1,
    state_placement,
)
transitions_take_ball_up = graph.createTransition(
    state_grasp_placement, state_ball_above_ground, "take-ball-up", 1, state_grasp
)
transitions_put_ball_down = graph.createTransition(
    state_ball_above_ground, state_grasp_placement, "put-ball-down", 1, state_grasp
)
transitions_take_ball_away = graph.createTransition(
    state_ball_above_ground, state_grasp, "take-ball-away", 1, state_grasp
)
transitions_approach_ground = graph.createTransition(
    state_grasp, state_ball_above_ground, "approach-ground", 1, state_grasp
)
transitions_transfer = graph.createTransition(
    state_grasp, state_grasp, "transfer", 1, state_grasp
)

joint2 = robot.model().getJointId("pokeball/root_joint")
joint1 = robot.model().getJointId("ur5/wrist_3_joint")
Id = SE3.Identity()

m = [
    False,
    False,
    True,
    True,
    True,
    False,
]
q = Quaternion(0, 0, 0, 1)
ballGround = SE3(q, np.array([0, 0, 0.15]))
pc = Transformation.create(
    "placementConstraint", robot.asPinDevice(), joint2, Id, ballGround, m
)
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
)
implicit_mask = [True, True, True]
implicitPlacementConstraint = Implicit.create(pc, cts, implicit_mask)


q = Quaternion(0.5, 0.5, -0.5, 0.5)
ballInGripper = SE3(q, np.array([0, 0.137, 0]))
m = Mask()
m[:] = (True,) * 6
pc = RelativeTransformation.create(
    "grasp", robot.asPinDevice(), joint1, joint2, ballInGripper, Id, m
)
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
)
implicitGraspConstraint = Implicit.create(pc, cts, m)


# Set constraints of nodes and edges
graph.addNumericalConstraint(state_placement, implicitPlacementConstraint)
graph.addNumericalConstraint(state_grasp, implicitGraspConstraint)

graph.maxIterations(100)
graph.errorThreshold(0.00001)

graph.initialize()

configurationShooter = problem.configurationShooter()
solver = BySubstitution(robot.configSpace())

for i in range(100):
    q = configurationShooter.shoot()
    res, config, err = graph.applyStateConstraints(state_placement, q)
    if res:
        print("after applying constraints: ", config)
        break
