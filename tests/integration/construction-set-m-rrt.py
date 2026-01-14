#!/usr/bin/env python

from math import pi, sqrt
import numpy as np
import re
import typing as T

from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
from pyhpp.manipulation import Device, Graph, Problem, urdf, ManipulationPlanner
from pyhpp.core import Straight, Progressive, ProgressiveProjector
from pyhpp.constraints import (
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
    LockedJoint,
)
from pinocchio import SE3, Quaternion

from pyhpp.manipulation.security_margins import SecurityMargins

from benchmark_utils import create_benchmark_parser, run_benchmark_main

parser = create_benchmark_parser("Construction Set M-RRT Benchmark")
parser.add_argument('--bigGraph', action='store_true',
    help="Whether constraint graph is generated with all the possible states. "
         "If unspecified, constraint graph only has a few states traversed by "
         "one safe path.")
args = parser.parse_args()


class StateName(object):
    noGrasp = 'free'
    
    def __init__(self, grasps):
        if isinstance(grasps, set):
            self.grasps = grasps.copy()
        elif isinstance(grasps, str):
            if grasps == self.noGrasp:
                self.grasps = set()
            else:
                g1 = [s.strip(' ') for s in grasps.split(':')]
                self.grasps = set([tuple(s.split(' grasps ')) for s in g1])
        else:
            raise TypeError('expecting a set of pairs (gripper, handle) or a string')
    
    def __str__(self):
        if self.grasps == set():
            return 'free'
        res = ""
        for g in self.grasps:
            res += g[0] + " grasps " + g[1] + " : "
        return res[:-3]
    
    def __eq__(self, other):
        return self.grasps == other.grasps
    
    def __ne__(self, other):
        return not self.__eq__(other)


ur3_urdf = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
ur3_srdf = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
cylinder_08_urdf = "package://hpp_environments/urdf/construction_set/cylinder_08.urdf"
cylinder_08_srdf = "package://hpp_environments/srdf/construction_set/cylinder_08.srdf"
sphere_urdf = "package://hpp_environments/urdf/construction_set/sphere.urdf"
sphere_srdf = "package://hpp_environments/srdf/construction_set/sphere.srdf"
ground_urdf = "package://hpp_environments/urdf/construction_set/ground.urdf"
ground_srdf = "package://hpp_environments/srdf/construction_set/ground.srdf"

nSphere = 2
nCylinder = 2

robot = Device('2ur5-sphere')

r0_pose = SE3(rotation=np.identity(3), translation=np.array([-.25, 0, 0]))
urdf.loadModel(robot, 0, "r0", "anchor", ur3_urdf, ur3_srdf, r0_pose)

r1_pose = SE3(Quaternion(0, 0, 0, 1), np.array([.25, 0, 0]))
urdf.loadModel(robot, 0, "r1", "anchor", ur3_urdf, ur3_srdf, r1_pose)

robot.setJointBounds('r0/shoulder_pan_joint', [-pi, 4])
robot.setJointBounds('r1/shoulder_pan_joint', [-pi, 4])
robot.setJointBounds('r0/shoulder_lift_joint', [-pi, 0])
robot.setJointBounds('r1/shoulder_lift_joint', [-pi, 0])
robot.setJointBounds('r0/elbow_joint', [-2.6, 2.6])
robot.setJointBounds('r1/elbow_joint', [-2.6, 2.6])

ground_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "ground", "anchor", ground_urdf, ground_srdf, ground_pose)

objects = list()
for i in range(nSphere):
    sphere_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, f'sphere{i}', "freeflyer", sphere_urdf, sphere_srdf, sphere_pose)
    robot.setJointBounds(
        f'sphere{i}/root_joint',
        [-1., 1., -1., 1., -.1, 1., -1.0001, 1.0001, -1.0001, 1.0001,
         -1.0001, 1.0001, -1.0001, 1.0001]
    )
    objects.append(f'sphere{i}')

for i in range(nCylinder):
    cylinder_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, f'cylinder{i}', "freeflyer", cylinder_08_urdf, cylinder_08_srdf, cylinder_pose)
    robot.setJointBounds(
        f'cylinder{i}/root_joint',
        [-1., 1., -1., 1., -.1, 1., -1.0001, 1.0001, -1.0001, 1.0001,
         -1.0001, 1.0001, -1.0001, 1.0001]
    )
    objects.append(f'cylinder{i}')

model = robot.model()

problem = Problem(robot)
problem.clearConfigValidations()
problem.addConfigValidation("CollisionValidation")

cg = Graph("assembly", robot, problem)
cg.errorThreshold(1e-4)
cg.maxIterations(40)

constraints = dict()

for i in range(nSphere):
    placementName = f"place_sphere{i}"
    Id = SE3.Identity()
    spherePlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.025]))
    joint = model.getJointId(f"sphere{i}/root_joint")
    
    pc = Transformation(
        placementName, robot, joint, Id, spherePlacement,
        [False, False, True, True, True, False],
    )
    cts = ComparisonTypes()
    cts[:] = (ComparisonType.EqualToZero, ComparisonType.EqualToZero, ComparisonType.EqualToZero)
    constraints[placementName] = Implicit(pc, cts, [True, True, True])
    
    pc_complement = Transformation(
        placementName + "/complement", robot, joint, Id, spherePlacement,
        [True, True, False, False, False, True],
    )
    cts_complement = ComparisonTypes()
    cts_complement[:] = (ComparisonType.Equality, ComparisonType.Equality, ComparisonType.Equality)
    constraints[placementName + "/complement"] = Implicit(pc_complement, cts_complement, [True, True, True])
    
    cts_hold = ComparisonTypes()
    cts_hold[:] = (
        ComparisonType.Equality, ComparisonType.Equality, ComparisonType.EqualToZero,
        ComparisonType.EqualToZero, ComparisonType.EqualToZero, ComparisonType.Equality,
    )
    constraints[placementName + "/hold"] = LockedJoint(
        robot, f"sphere{i}/root_joint", np.array([0, 0, 0.025, 0, 0, 0, 1]), cts_hold
    )
    
    cg.registerConstraints(
        constraints[placementName],
        constraints[placementName + "/complement"],
        constraints[placementName + "/hold"],
    )
    
    preplacementName = f"preplace_sphere{i}"
    spherePrePlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.075]))
    pc_pre = Transformation(
        preplacementName, robot, joint, Id, spherePrePlacement,
        [False, False, True, True, True, False],
    )
    constraints[preplacementName] = Implicit(pc_pre, cts, [True, True, True])

for i in range(nCylinder):
    placementName = f"place_cylinder{i}"
    Id = SE3.Identity()
    cylinderPlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.025]))
    joint = model.getJointId(f"cylinder{i}/root_joint")
    
    pc = Transformation(
        placementName, robot, joint, Id, cylinderPlacement,
        [False, False, True, True, True, False],
    )
    cts = ComparisonTypes()
    cts[:] = (ComparisonType.EqualToZero, ComparisonType.EqualToZero, ComparisonType.EqualToZero)
    constraints[placementName] = Implicit(pc, cts, [True, True, True])
    
    pc_complement = Transformation(
        placementName + "/complement", robot, joint, Id, cylinderPlacement,
        [True, True, False, False, False, True],
    )
    cts_complement = ComparisonTypes()
    cts_complement[:] = (ComparisonType.Equality, ComparisonType.Equality, ComparisonType.Equality)
    constraints[placementName + "/complement"] = Implicit(pc_complement, cts_complement, [True, True, True])
    
    cts_hold = ComparisonTypes()
    cts_hold[:] = (
        ComparisonType.Equality, ComparisonType.Equality, ComparisonType.EqualToZero,
        ComparisonType.EqualToZero, ComparisonType.EqualToZero, ComparisonType.Equality,
    )
    constraints[placementName + "/hold"] = LockedJoint(
        robot, f"cylinder{i}/root_joint", np.array([0, 0, 0.025, 0, 0, 0, 1]), cts_hold
    )
    
    cg.registerConstraints(
        constraints[placementName],
        constraints[placementName + "/complement"],
        constraints[placementName + "/hold"],
    )
    
    preplacementName = f"preplace_cylinder{i}"
    cylinderPrePlacement = SE3(Quaternion(1, 0, 0, 0), np.array([0, 0, 0.075]))
    pc_pre = Transformation(
        preplacementName, robot, joint, Id, cylinderPrePlacement,
        [False, False, True, True, True, False],
    )
    constraints[preplacementName] = Implicit(pc_pre, cts, [True, True, True])

grippers = [f'cylinder{i}/magnet0' for i in range(nCylinder)]
grippers += [f'cylinder{i}/magnet1' for i in range(nCylinder)]
grippers += [f"r{i}/gripper" for i in range(2)]

handlesPerObjects = [[f'sphere{i}/handle', f'sphere{i}/magnet'] for i in range(nSphere)]
handlesPerObjects += [[f'cylinder{i}/handle'] for i in range(nCylinder)]

shapesPerObject = [[] for o in objects]

allHandles = [handle for objHandles in handlesPerObjects for handle in objHandles]


def makeRule(grasps):
    _grippers = list()
    _handles = list()
    for (g, h) in grasps:
        _grippers.append(g)
        _handles.append(h)
    for g in grippers:
        if not g in _grippers:
            _grippers.append(g)
    _handles += (len(_grippers) - len(_handles)) * ['^$']
    return Rule(grippers=_grippers, handles=_handles, link=True)


def forbidExcept(g: str, h: T.List[str]) -> T.List[Rule]:
    gRegex = re.compile(g)
    hRegex = [re.compile(handle) for handle in h]
    forbiddenList: T.List[Rule] = list()
    idForbidGrippers = [i for i in range(len(grippers)) if gRegex.match(grippers[i])]
    forbidHandles = [handle for handle in allHandles 
                     if not any([handlePattern.match(handle) for handlePattern in hRegex])]
    for id in idForbidGrippers:
        for handle in forbidHandles:
            _handles = [handle if i == id else '.*' for i in range(len(grippers))]
            forbiddenList.append(Rule(grippers=grippers, handles=_handles, link=False))
    return forbiddenList


q0_r0 = [pi/6, -pi/2, pi/2, 0, 0, 0]
q0_r1 = q0_r0[::]

q0_spheres = list()
i = 0
y = 0.04
while i < nSphere:
    q0_spheres.append([-.1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
    i += 1
    y = -y

q0_cylinders = list()
i = 0
y = -0.04
while i < nCylinder:
    q0_cylinders.append([0.45 + .1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
    i += 1
    y = -y

q0 = np.array(q0_r0 + q0_r1 + sum(q0_spheres, []) + sum(q0_cylinders, []))
if not args.bigGraph:
    nodes = list()
    rules = list()
    grasps = set()
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.add(('r0/gripper', 'sphere0/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.add(('r1/gripper', 'cylinder0/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.add(('cylinder0/magnet0', 'sphere0/magnet'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.remove(('r0/gripper', 'sphere0/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.add(('r0/gripper', 'sphere1/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.add(('cylinder0/magnet1', 'sphere1/magnet'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.remove(('r0/gripper', 'sphere1/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    grasps.remove(('r1/gripper', 'cylinder0/handle'))
    nodes.append(StateName(grasps))
    rules.append(makeRule(grasps=grasps))
    
    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Progressive(robot, 0.02)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.05
    )
    
    factory = ConstraintGraphFactory(cg, constraints)
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, shapesPerObject)
    factory.setRules(rules)
    factory.generate()
    
    sm = SecurityMargins(problem, factory, ["r0", "r1", "sphere0", "sphere1",
                                            "cylinder0", "cylinder1"], robot)
    sm.defaultMargin = 0.02
    sm.apply()
    
    cg.initialize()
    
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.05
    )
    
    nodes.append(nodes[-1])

else:
    rules = list()
    
    rules.extend(forbidExcept('^r0/gripper$', ['^sphere\\d*/handle$']))
    rules.extend(forbidExcept('^r1/gripper$', ['^cylinder0*/handle$']))
    rules.extend(forbidExcept('^cylinder0/magnet\\d*$', ['^sphere\\d*/magnet$']))
    rules.extend(forbidExcept('^cylinder[1-9][0-9]*.*$', ['^$']))
    
    rules.append(Rule(grippers=grippers, handles=[".*"] * len(grippers), link=True))
    
    problem.steeringMethod = Straight(problem)
    problem.pathValidation = Progressive(robot, 0.02)
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.05
    )
    
    factory = ConstraintGraphFactory(cg, constraints)
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, shapesPerObject)
    factory.setRules(rules)
    factory.generate()
    
    sm = SecurityMargins(problem, factory, ["r0", "r1", "sphere0", "sphere1",
                                            "cylinder0", "cylinder1"], robot)
    sm.defaultMargin = 0.02
    sm.apply()
    
    cg.initialize()
    
    problem.pathProjector = ProgressiveProjector(
        problem.distance(), problem.steeringMethod, 0.05
    )

assert (nCylinder == 2 and nSphere == 2)
c = sqrt(2) / 2
q_goal = np.array(q0_r0 + q0_r1 + [-0.06202136144745322, -0.15, 0.025, c, 0, -c, 0,
                                   0.06202136144745322, -0.15, 0.025, c, 0, c, 0,
                                   0, -0.15, 0.025, 0, 0, 0, 1,
                                   0.5, -0.08, 0.025, 0, 0, 0, 1])

problem.initConfig(q0)
problem.addGoalConfig(q_goal)
problem.constraintGraph(cg)

if args.display:
    from pyhpp_plot import show_graph
    show_graph(cg)

manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(5000)

# Run benchmark using shared utilities
if args.N > 0:
    run_benchmark_main(
        planner=manipulationPlanner,
        problem=problem,
        q_init=q0,
        q_goal=q_goal,
        num_iterations=args.N,
    )