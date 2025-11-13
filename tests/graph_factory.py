from math import sqrt
import numpy as np

from pyhpp.manipulation.constraint_graph_factory import Rule, ConstraintGraphFactory
from pyhpp.manipulation import Device, urdf, Graph, Problem
from pyhpp.constraints import LockedJoint
from pinocchio import SE3

# Robot and environment file paths
pr2_urdf = "package://example-robot-data/robots/pr2_description/urdf/pr2.urdf"
pr2_srdf = (
    "package://example-robot-data/robots/pr2_description/srdf/pr2_manipulation.srdf"
)
box_urdf = "package://hpp_tutorial/urdf/box.urdf"
box_srdf = "package://hpp_tutorial/srdf/box.srdf"
kitchen_urdf = "package://hpp_tutorial/urdf/kitchen_area.urdf"
kitchen_srdf = "package://hpp_tutorial/srdf/kitchen_area.srdf"

robot = Device("pr2-box")

# Load PR2 robot
pr2_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(robot, 0, "pr2", "freeflyer", pr2_urdf, pr2_srdf, pr2_pose)

# Load box to be manipulated
box_pose = SE3(rotation=np.identity(3), translation=np.array([-2.5, -4, 0.746]))
urdf.loadModel(robot, 0, "box", "freeflyer", box_urdf, box_srdf, box_pose)

# Load kitchen environment
kitchen_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
urdf.loadModel(
    robot, 0, "kitchen_area", "anchor", kitchen_urdf, kitchen_srdf, kitchen_pose
)

model = robot.model()

robot.setJointBounds("pr2/root_joint", [-5, -2, -5.2, -2.7])
robot.setJointBounds("box/root_joint", [-5.1, -2, -5.2, -2.7, 0, 1.5])

# Create problem
problem = Problem(robot)
print(model.names)

# Generate initial and goal configuration.
rankInConfiguration = dict()
current_rank = 0
for joint_id in range(1, model.njoints):
    joint_name = model.names[joint_id]
    joint = model.joints[joint_id]
    rankInConfiguration[joint_name[0:]] = current_rank
    current_rank += joint.nq


q_init = robot.currentConfiguration()
rank = rankInConfiguration["pr2/l_gripper_l_finger_joint"]
q_init[rank] = 0.5
rank = rankInConfiguration["pr2/l_gripper_r_finger_joint"]
q_init[rank] = 0.5
q_init[0:2] = [-3.2, -4]
rank = rankInConfiguration["pr2/torso_lift_joint"]
q_init[rank] = 0.2
rank = rankInConfiguration["box/root_joint"]
q_init[rank : rank + 3] = [-2.5, -4, 0.746]
q_init[rank + 3 : rank + 7] = [0, -sqrt(2) / 2, 0, sqrt(2) / 2]

q_goal = q_init[::]
q_goal[0:2] = [-3.2, -4]
rank = rankInConfiguration["box/root_joint"]
q_goal[rank : rank + 3] = [-2.5, -4.5, 0.746]

# Create constraints
ll = LockedJoint(
    robot, "pr2/l_gripper_l_finger_joint", np.array([0.5])
)
lr = LockedJoint(
    robot, "pr2/l_gripper_r_finger_joint", np.array([0.5])
)

# Create the constraint graph
grippers = ["pr2/l_gripper"]
objects = ["box"]
handlesPerObject = [["box/handle2"]]
contactSurfacesPerObject = [["box/box_surface"]]
envContactSurfaces = ["kitchen_area/pancake_table_table_top"]
rules = [Rule([".*"], [".*"], True)]

cg = Graph("graph", robot, problem)
cg.maxIterations(100)
cg.errorThreshold(0.0001)
factory = ConstraintGraphFactory(cg)

factory.setGrippers(grippers)
factory.environmentContacts(envContactSurfaces)
factory.setObjects(objects, handlesPerObject, contactSurfacesPerObject)
factory.setRules(rules)
factory.generate()

cg.addNumericalConstraintsToGraph([ll, lr])

cg.initialize()

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)

print("Script completed!")
