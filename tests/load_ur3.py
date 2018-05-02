from pyhpp.pinocchio import Device, urdf

robot = Device.create('ur3')
urdf.loadRobotModel (robot, "anchor", "ur_description", "ur3", "_gripper", "_gripper")

q = robot.currentConfiguration()
robot.currentConfiguration (q)

from pyhpp.constraints import Position
from pinocchio import SE3, StdVec_Bool as Mask
import numpy as np

m = Mask()
m[:] = (True,)*3
Id = SE3.Identity()
pc = Position.create ("position", robot,
        robot.model().getJointId("wrist_3_joint"),
        Id, Id, m)
print pc.inputSize

qa = np.zeros((6, 1))

v = pc(qa)
print v.space().name(), ':',  v.vector()

J = pc.jacobian (q)
print J
