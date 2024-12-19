import numpy as np
from pinocchio import SE3
from pinocchio import StdVec_Bool as Mask
from pyhpp.constraints import Position
from pyhpp.pinocchio import Device, LiegroupElement, urdf

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
