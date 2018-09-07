from pyhpp.pinocchio import Device, urdf

robot = Device.create('ur3')
urdf.loadRobotModel (robot, "anchor", "ur_description", "ur3", "_gripper", "_gripper")

from pyhpp.constraints import Position, ComparisonTypes, ComparisonType, BySubstitution, segment
from pinocchio import SE3, StdVec_Bool as Mask
import numpy as np

m = Mask()
m[:] = (True,)*3
Id = SE3.Identity()
pc = Position.create ("position", robot,
        robot.model().getJointId("wrist_3_joint"),
        Id, Id, m)

solver = BySubstitution (robot.model().nq,robot.model().nv)
solver.add (pc, 0)
cts = ComparisonTypes ()
cts[:] = (ComparisonType.EqualToZero, ComparisonType.EqualToZero, ComparisonType.Equality)
solver.add (pc, 0, cts)

print solver
print solver.explicitSolver()

# This only tests the call to add.
# The inputs are not valid so the solver
# will not be able to solve anything.
solver.explicitSolver().add (pc,
        [ segment(0,2), ], [ segment(2,4), ],
        [ segment(0,2), ], [ segment(2,4), ],
        cts)
solver.explicitSolverHasChanged()
