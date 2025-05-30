from math import pi
import numpy as np
from pinocchio import SE3
from pyhpp.manipulation import Device, urdf
from pyhpp.gepetto import Viewer

robot = Device("construction-set")

#Create viewer
viewer = Viewer("construction_set", robot)

# Load two UR3 robots
urdfFilename = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
srdfFilename = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"

r0_pose = SE3(rotation = np.identity(3), translation = np.array([-0.25, 0, 0]))
r1_pose = SE3(rotation = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), translation = np.array([ 0.25, 0, 0]))

#urdf.loadModel(robot, 0, "r0", "anchor", urdfFilename, srdfFilename, r0_pose)
#urdf.loadModel(robot, 0, "r1", "anchor", urdfFilename, srdfFilename, r1_pose)
viewer.addURDFToScene(0, "r0", "anchor", urdfFilename, srdfFilename, r0_pose)
viewer.addURDFToScene(0, "r1", "anchor", urdfFilename, srdfFilename, r1_pose)

# Load environment
urdfFilename = "package://hpp_environments/urdf/construction_set/ground.urdf"
srdfFilename = "package://hpp_environments/srdf/construction_set/ground.srdf"
#urdf.loadModel(robot, 0, "ground", "anchor", urdfFilename, srdfFilename, SE3.Identity())
viewer.addURDFToScene(0, "ground", "anchor", urdfFilename, srdfFilename, SE3.Identity())

q = np.array([pi/6, -pi/2, pi/2, 0, 0, 0,] + [pi/6, -pi/2, pi/2, 0, 0, 0,])
viewer.applyConfiguration(q)

# Load spheres
nSphere = 2
nCylinder = 2

objects = list ()
urdfFilename = "package://hpp_environments/urdf/construction_set/sphere.urdf"
srdfFilename = "package://hpp_environments/srdf/construction_set/sphere.srdf"
for i in range (nSphere):
    #urdf.loadModel(robot, 0, f"sphere{i}", "freeflyer", urdfFilename, srdfFilename, SE3.Identity())
    viewer.addURDFToScene(0, f"sphere{i}", "freeflyer", urdfFilename, srdfFilename, SE3.Identity())
    objects.append (f"sphere{i}")

# Load cylinders
urdfFilename = "package://hpp_environments/urdf/construction_set/cylinder_08.urdf"
srdfFilename = "package://hpp_environments/srdf/construction_set/cylinder_08.srdf"
for i in range (nCylinder):
    #urdf.loadModel(robot, 0, f"cylinder{i}", "freeflyer", urdfFilename, srdfFilename,SE3.Identity())
    viewer.addURDFToScene(0, f"cylinder{i}", "freeflyer", urdfFilename, srdfFilename,SE3.Identity())
    objects.append (f"cylinder{i}")

# Set joint bounds
# Set robot joint bounds
jointBounds =  {'r0/shoulder_pan_joint': [-pi, 4],
                'r1/shoulder_pan_joint': [-pi, 4],
                'r0/shoulder_lift_joint': [-pi, 0],
                'r1/shoulder_lift_joint': [-pi, 0],
                'r0/elbow_joint': [-2.6, 2.6],
                'r1/elbow_joint': [-2.6, 2.6]}
m = robot.model()
lowerLimit = m.lowerPositionLimit
upperLimit = m.upperPositionLimit
for jn, [lower, upper] in jointBounds.items():
    lowerLimit[m.getJointId(jn)] = lower
    upperLimit[m.getJointId(jn)] = upper

for i in range (nSphere):
    ij = m.getJointId(f"sphere{i}/root_joint")
    iq = m.idx_qs[ij]
    nq = m.nqs[ij]
    lowerLimit[iq:iq+nq] = \
               [-1., -1., -.1, -1.0001, -1.0001, -1.0001, -1.0001]
    upperLimit[iq:iq+nq] = \
               [ 1.,  1.,  .1,  1.0001,  1.0001,  1.0001,  1.0001]

for i in range (nCylinder):
    ij = m.getJointId(f"cylinder{i}/root_joint")
    iq = m.idx_qs[ij]
    nq = m.nqs[ij]
    lowerLimit[iq:iq+nq] = \
               [-1., -1., -.1, -1.0001, -1.0001, -1.0001, -1.0001]
    upperLimit[iq:iq+nq] = \
               [ 1.,  1.,  .1,  1.0001,  1.0001,  1.0001,  1.0001]

m.lowerPositionLimit = lowerLimit
m.upperPositionLimit = upperLimit

grippers = robot.grippers()
grippers["r0/gripper"].localPosition

handles = robot.handles()
handles["sphere0/magnet"].localPosition

h = handles["sphere1/magnet"]
g = grippers["cylinder0/magnet1"]

c = h.createGrasp(g, "sphere1/magnet grasps cylinder0/magnet1")
