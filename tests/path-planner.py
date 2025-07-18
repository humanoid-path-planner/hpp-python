import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import (
    Problem,
    DiffusingPlanner,
    BiRRTPlanner,
    VisibilityPrmPlanner,
    BiRrtStar,
    kPrmStar,
)

urdfFilename = "package://example-robot-data/robots/pr2_description/urdf/pr2.urdf"
srdfFilename = "package://example-robot-data/robots/pr2_description/srdf/pr2.srdf"
rootJointType = "planar"

# Initialize robot and viewer
robot = Device.create("ur2")
urdf.loadModel(robot, 0, "r0", rootJointType, urdfFilename, srdfFilename, SE3.Identity())

# Define initial and goal configurations
q_init = np.array(robot.currentConfiguration())
q_goal = np.array(q_init[::].copy())
model = robot.model()

# Set root_joint bounds only
lowerLimit = model.lowerPositionLimit
upperLimit = model.upperPositionLimit

# Set root_joint bounds specifically
root_joint_bounds = [-4, -3, -5, -3]  # [x_lower, x_upper, y_lower, y_upper]
ij = model.getJointId("r0/root_joint")
iq = model.idx_qs[ij]

# Apply bounds (assuming first 2 DOF are x,y position)
lowerLimit[iq] = -4
upperLimit[iq] = -3
lowerLimit[iq + 1] = -5
upperLimit[iq + 1] = -3

rankInConfiguration = dict()
current_rank = 0
for joint_id in range(1, model.njoints):
    joint_name = model.names[joint_id]
    joint = model.joints[joint_id]
    rankInConfiguration[joint_name[3:]] = current_rank

    current_rank += joint.nq

q_init[0:2] = [-3.2, -4]

rank = rankInConfiguration["torso_lift_joint"]
q_init[rank] = 0.2
q_goal[0:2] = [-3.2, -4]
rank = rankInConfiguration["l_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = rankInConfiguration["l_elbow_flex_joint"]
q_goal[rank] = -0.5
rank = rankInConfiguration["r_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = rankInConfiguration["r_elbow_flex_joint"]
q_goal[rank] = -0.5

#urdf.loadModel(robot, 0, "kitchen", "anchor", "package://hpp_environments/urdf/kitchen_area_obstacle.urdf", "", SE3.Identity())

problem = Problem(robot)
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)

diffusingPlanner = DiffusingPlanner(problem)
biRRTPlanner = BiRRTPlanner(problem)
visibilityPrmPlanner = VisibilityPrmPlanner(problem)
biRrtStar = BiRrtStar(problem)
kPrmStar_inst = kPrmStar(problem)

print(diffusingPlanner.maxIterations())

path = biRRTPlanner.solve()
# viewer.displayPath(path, 0.001, 50)

# path = diffusingPlanner.solve()
# viewer.displayPath(path, 0.001, 50)

# path = visibilityPrmPlanner.solve() infinite search
# viewer.displayPath(path, 0.001, 50)

# path = biRrtStar.solve()
# viewer.displayPath(path, 0.001, 50)

# path = kPrmStar_inst.solve()  infinite search
# viewer.displayPath(path, 0.001, 50)

# roadmap = Roadmap.create(problem.distance(), robot)
# roadmap.initNode(q_init)
# roadmap.addGoalNode(q_goal)
# searchInRoadmap = SearchInRoadmap(problem, roadmap)
# searchInRoadmap.solve() SearchInRoadmap: no goal configuration in the connected componentof initial configuration. A* fails

# planAndOptimize = PlanAndOptimize(diffusingPlanner)
# viewer.displayPath(planAndOptimize.solve(), 0.001, 50)
