#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import datetime as dt

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
from pyhpp.manipulation import Device, Graph, Problem, urdf, ManipulationPlanner
from pyhpp.core import Dichotomy, Straight, ProgressiveProjector
from pyhpp.constraints import LockedJoint
from pyhpp.core.static_stability_constraint_factory import (
    StaticStabilityConstraintsFactory,
)
from pinocchio import SE3

parser = ArgumentParser()
parser.add_argument("-N", default=0, type=int)
parser.add_argument("--display", action="store_true")
parser.add_argument("--run", action="store_true")
args = parser.parse_args()

# Robot and object file paths
romeo_urdf = "package://example-robot-data/robots/romeo_description/urdf/romeo.urdf"
romeo_srdf_moveit = (
    "package://example-robot-data/robots/romeo_description/srdf/romeo_moveit.srdf"
)
placard_urdf = "package://hpp_environments/urdf/placard.urdf"
placard_srdf = "package://hpp_environments/srdf/placard.srdf"

robot = Device("romeo-placard")

# Load Romeo robot
romeo_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(
    robot, 0, "romeo", "freeflyer", romeo_urdf, romeo_srdf_moveit, romeo_pose
)

robot.setJointBounds(
    "romeo/root_joint", [-1, 1, -1, 1, 0, 2, -2.0, 2, -2.0, 2, -2.0, 2, -2.0, 2]
)

# Load placard
placard_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(
    robot, 0, "placard", "freeflyer", placard_urdf, placard_srdf, placard_pose
)

robot.setJointBounds(
    "placard/root_joint", [-1, 1, -1, 1, 0, 1.5, -2.0, 2, -2.0, 2, -2.0, 2, -2.0, 2]
)

model = robot.model()

problem = Problem(robot)

problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

cg = Graph("graph", robot, problem)
cg.errorThreshold(2e-4)
cg.maxIterations(40)

constraints = dict()
graphConstraints = dict()

leftHandOpen = {
    "LHand": 1,
    "LFinger12": 1.06,
    "LFinger13": 1.06,
    "LFinger21": 1.06,
    "LFinger22": 1.06,
    "LFinger23": 1.06,
    "LFinger31": 1.06,
    "LFinger32": 1.06,
    "LFinger33": 1.06,
    "LThumb1": 0,
    "LThumb2": 1.06,
    "LThumb3": 1.06,
}

rightHandOpen = {
    "RHand": 1,
    "RFinger12": 1.06,
    "RFinger13": 1.06,
    "RFinger21": 1.06,
    "RFinger22": 1.06,
    "RFinger23": 1.06,
    "RFinger31": 1.06,
    "RFinger32": 1.06,
    "RFinger33": 1.06,
    "RThumb1": 0,
    "RThumb2": 1.06,
    "RThumb3": 1.06,
}


# Lock left hand
locklhand = list()
for j, v in leftHandOpen.items():
    joint_name = "romeo/" + j
    locklhand.append(joint_name)
    if type(v) is float or type(v) is int:
        val = np.array([v])
    else:
        val = np.array(v)
    cs = LockedJoint(robot, joint_name, val)
    constraints[joint_name] = cs
    graphConstraints[joint_name] = cs

# Lock right hand
lockrhand = list()
for j, v in rightHandOpen.items():
    joint_name = "romeo/" + j
    lockrhand.append(joint_name)
    if type(v) is float or type(v) is int:
        val = np.array([v])
    else:
        val = np.array(v)
    cs = LockedJoint(robot, joint_name, val)
    constraints[joint_name] = cs
    graphConstraints[joint_name] = cs

lockHands = lockrhand + locklhand

# Create static stability constraint
q = robot.currentConfiguration()
placard_rank = model.idx_qs[model.getJointId("placard/root_joint")]
q[placard_rank : placard_rank + 3] = [0.4, 0, 1.2]

problem.addPartialCom("romeo", ["romeo/root_joint"])

leftAnkle = "romeo/LAnkleRoll"
rightAnkle = "romeo/RAnkleRoll"

factory = StaticStabilityConstraintsFactory(problem, robot)
balanceConstraintsDict = factory.createStaticStabilityConstraint(
    "balance/", "romeo", leftAnkle, rightAnkle, q
)

balanceConstraints = [
    balanceConstraintsDict.get("balance/pose-left-foot"),
    balanceConstraintsDict.get("balance/pose-right-foot"),
    balanceConstraintsDict.get("balance/relative-com"),
]
balanceConstraints = [c for c in balanceConstraints if c is not None]

for name, constraint in balanceConstraintsDict.items():
    if constraint not in balanceConstraints:
        constraints[name] = constraint
        graphConstraints[name] = constraint

# Build graph
grippers = ["romeo/r_hand", "romeo/l_hand"]
handlesPerObjects = [["placard/low", "placard/high"]]

rules = [
    Rule(["romeo/l_hand", "romeo/r_hand"], ["placard/low", ""], True),
    Rule(["romeo/l_hand", "romeo/r_hand"], ["", "placard/high"], True),
    Rule(["romeo/l_hand", "romeo/r_hand"], ["placard/low", "placard/high"], True),
]

factory_cg = ConstraintGraphFactory(cg, constraints)
factory_cg.setGrippers(grippers)
factory_cg.setObjects(["placard"], handlesPerObjects, [[]])
factory_cg.setRules(rules)
factory_cg.generate()

# Add balance constraints and locked hands to graph
all_graph_constraints = list(graphConstraints.values()) + balanceConstraints
cg.addNumericalConstraintsToGraph(all_graph_constraints)
cg.initialize()

# Define initial and final configurations
q_goal = np.array(
    [
        -0.003429678026293006,
        7.761615492429529e-05,
        0.8333148411182841,
        -0.08000440760954532,
        0.06905332841243099,
        -0.09070086400314036,
        0.9902546570793265,
        0.2097693637044623,
        0.19739743868699455,
        -0.6079135018296973,
        0.8508704420155889,
        -0.39897628829947995,
        -0.05274298289004072,
        0.20772797293264825,
        0.1846394290733244,
        -0.49824886682709824,
        0.5042013065348324,
        -0.16158420369261683,
        -0.039828502509861335,
        -0.3827070014985058,
        -0.24118425356319423,
        1.0157846623463191,
        0.5637424355124602,
        -1.3378817283780955,
        -1.3151786907256797,
        -0.392409481224193,
        0.11332560818107676,
        1.06,
        1.06,
        1.06,
        1.06,
        1.06,
        1.06,
        1.0,
        1.06,
        1.06,
        -1.06,
        1.06,
        1.06,
        0.35936687035487364,
        -0.32595302056157444,
        -0.33115291290191723,
        0.20387672048126043,
        0.9007626913161502,
        -0.39038645767349395,
        0.31725226129015516,
        1.5475253831101246,
        -0.0104572058777634,
        0.32681856374063933,
        0.24476959944940427,
        1.06,
        1.06,
        1.06,
        1.06,
        1.06,
        1.06,
        1.0,
        1.06,
        1.06,
        -1.06,
        1.06,
        1.06,
        0.412075621240969,
        0.020809907186176854,
        1.056724788359247,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
)
q_init = q_goal.copy()
q_init[placard_rank + 3 : placard_rank + 7] = [0, 0, 1, 0]

n = "romeo/l_hand grasps placard/low"
state = cg.getState(n)
res, q_init_proj, err = cg.applyStateConstraints(state, q_init)
if not res:
    raise RuntimeError("Failed to project initial configuration.")
res, q_goal_proj, err = cg.applyStateConstraints(state, q_goal)
if not res:
    raise RuntimeError("Failed to project goal configuration.")

problem.steeringMethod = Straight(problem)
problem.pathValidation = Dichotomy(robot, 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod, 0.05
)

problem.initConfig(q_init_proj)
problem.addGoalConfig(q_goal_proj)
problem.constraintGraph(cg)

manipulationPlanner = ManipulationPlanner(problem)
# Run benchmark
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
