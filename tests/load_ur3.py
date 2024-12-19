from pyhpp.pinocchio import Device, urdf, LiegroupElement

robot = Device.create("ur3")
urdf.loadRobotModel(
    robot,
    "anchor",
    "example-robot-data/robots/ur_description",
    "ur3",
    "_gripper",
    "_gripper",
)

q = robot.currentConfiguration()
robot.currentConfiguration(q)

from pyhpp.constraints import Position
from pinocchio import SE3, StdVec_Bool as Mask
import numpy as np

m = Mask()
m[:] = (True,) * 3
Id = SE3.Identity()
pc = Position.create(
    "position", robot, robot.model().getJointId("wrist_3_joint"), Id, Id, m
)
print(pc)

qa = np.zeros((pc.ni, 1))

# C++ API
v = LiegroupElement(pc.outputSpace())
pc.value(v, qa)
print(f"{v.space().name()}: {v.vector().T}")

J = np.zeros((pc.ndo, pc.ndi))
pc.jacobian(J, q)
print(J[:, 0:4])

# Pythonic API
v = pc(qa)
J = pc.J(q)
