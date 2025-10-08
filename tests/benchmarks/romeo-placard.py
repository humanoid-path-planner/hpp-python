#!/usr/bin/env python

from argparse import ArgumentParser
import datetime as dt
import numpy as np

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation import Device, Graph, Problem, urdf, ManipulationPlanner

from pinocchio import SE3

parser = ArgumentParser()
parser.add_argument("-N", default=20, type=int)
parser.add_argument("--display", action="store_true")
parser.add_argument("--run", action="store_true")
args = parser.parse_args()

romeo_urdf = "package://example-robot-data/robots/romeo_description/urdf/romeo.urdf"
romeo_srdf = (
    "package://example-robot-data/robots/romeo_description/srdf/romeo_moveit.srdf"
)

placard_urdf = "package://hpp_environments/urdf/placard.urdf"
placard_srdf = "package://hpp_environments/srdf/placard.srdf"

robot = Device("romeo")

# Load Romeo robot
romeo_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "romeo", "anchor", romeo_urdf, romeo_srdf, romeo_pose)
for name in robot.model().names:
    print(name)
robot.setJointBounds(
    "romeo/root_joint", [-1, 1, -1, 1, 0, 2, -2.0, 2, -2.0, 2, -2.0, 2, -2.0, 2]
)

# Load placard object
placard_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(
    robot,
    0,
    "placard",
    "freeflyer",
    placard_urdf,
    placard_srdf,
    placard_pose,
)

robot.setJointBounds(
    "placard/root_joint", [-1, 1, -1, 1, 0, 1.5, -2.0, 2, -2.0, 2, -2.0, 2, -2.0, 2]
)

problem = Problem(robot)
cg = Graph("graph", robot, problem)

problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

# Set error threshold and max iterations
cg.errorThreshold(2e-4)
cg.maxIterations(40)

constraints = dict()

# TODO: Lock both hands - need to find equivalent of robot.leftHandOpen and robot.rightHandOpen
# In the old API, these were dictionaries with joint names and values
# For now, creating placeholder structure
locklhand = []
lockrhand = []

# TODO: Create locked joints for hands
# Example structure (needs actual joint names and values):
# leftHandOpen = {'LFinger11': 0.0, 'LFinger12': 0.0, ...}
# rightHandOpen = {'RFinger11': 0.0, 'RFinger12': 0.0, ...}
# For each joint in these dicts, create LockedJoint constraint

lockHands = lockrhand + locklhand

# TODO: Create static stability constraint
# In old API this was:
# robot.createStaticStabilityConstraint('balance/', 'romeo', leftAnkle, rightAnkle, q)
# Need to find equivalent in new API using COM constraints and foot placement

# Placeholder for balance constraints names
balanceConstraints = [
    "balance/pose-left-foot",
    "balance/pose-right-foot",
    "balance/relative-com",
]

# TODO: Actually create these constraints using the new API
# Will need Transformation.create() for foot poses
# And COM constraint for relative-com

# Get initial configuration
# TODO: Need equivalent of robot.getInitialConfig()
q = np.zeros(robot.configSize())  # Placeholder

r = robot.model().getJointId("placard/root_joint")
# TODO: This rank calculation might be different in new API
# In old API: placard.rank = vf.robot.rankInConfiguration[name + '/root_joint']
# May need to use robot.model().idx_qs[joint_id]

q_init = [
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

q_goal = q_init[::]
# TODO: Adjust goal configuration for placard position
# q_goal[r+3:r+7] = [0, 0, 1, 0]

# Define grippers and handles
grippers = ["romeo/r_hand", "romeo/l_hand"]
handlesPerObjects = [["placard/low", "placard/high"]]
contactsPerObjects = [[]]

# TODO: In the old API, there were Rules to define valid grasping combinations
# rules = [Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", ""], True),
#          Rule (["romeo/l_hand","romeo/r_hand",], ["", "placard/high"], True),
#          Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", "placard/high"], True)]
# Need to check if ConstraintGraphFactory in new API supports setRules() or equivalent

factory = ConstraintGraphFactory(cg, constraints)
factory.setGrippers(grippers)
factory.setObjects(["placard"], handlesPerObjects, contactsPerObjects)

# TODO: Check if factory.setRules() exists in new API
# factory.setRules(rules)

factory.generate()

# TODO: Add common constraints (balance + locked hands) to all nodes/edges
# In old API: cg.addConstraints(graph=True, constraints=commonConstraints)
# Need to find equivalent in new API

cg.initialize()

# TODO: Apply node constraints to project configurations
# In old API: res, q_init, err = cg.applyNodeConstraints(n, q_init)
# Need equivalent in new API

# TODO: Set up path projector
# In old API: ps.selectPathProjector("Progressive", .05)
# In new API seen: problem.pathProjector = createProgressiveProjector(...)
# But need to check imports and exact usage

problem.initConfig(np.array(q_init))
problem.addGoalConfig(np.array(q_goal))
problem.constraintGraph(cg)

manipulationPlanner = ManipulationPlanner(problem)
# TODO: Default max iterations? Or need to set?

# Run benchmark
totalTime = dt.timedelta(0)
totalNumberNodes = 0
success = 0
for i in range(args.N):
    try:
        manipulationPlanner.roadmap().clear()
        # TODO: Equivalent of ps.resetGoalConfigs() if needed
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

# TODO: Display and path player functionality
# if args.display:
#     # Need equivalent of ViewerFactory and PathPlayer in new API
#     pass

# TODO: Benchmarking constraints section
# This entire section needs conversion:
# - robot.shootRandomConfig() equivalent
# - Creating grasp constraints
# - Benchmarking implicit vs explicit constraints
# - The benchConstraints function needs complete rewrite for new API

# Placeholder for constraint benchmarking
N_bench = 10000
qs = [None] * N_bench

# TODO: Implement random config generation
# for i in range(N_bench):
#     qs[i] = robot.shootRandomConfig()  # Need equivalent

# TODO: Create grasp constraints
# implicitGraspConstraints = dict()
# explicitGraspConstraints = dict()
# This section used createTransformationConstraint and cg.createGrasp
# Need to convert to new API using Transformation.create() and Implicit.create()

# TODO: Benchmark function needs complete rewrite
# def benchConstraints(constraints, lockDofs):
#     # Old API used ps.client.basic.problem.resetConstraints()
#     # Need equivalent in new API
#     pass

print("\nTODO: Constraint benchmarking section not yet converted")
