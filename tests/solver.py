from pyhpp.pinocchio import Device, urdf

from pyhpp.constraints import (
    Position,
    ComparisonTypes,
    ComparisonType,
    BySubstitution,
    segment,
    Implicit,
    Explicit,
)
from pinocchio import SE3, StdVec_Bool as Mask

robot = Device.create("ur3")
urdf.loadRobotModel(
    robot,
    "anchor",
    "example-robot-data/robots/ur_description",
    "ur3",
    "_gripper",
    "_gripper",
)

m = Mask()
m[:] = (True,) * 3
Id = SE3.Identity()
pc = Position.create(
    "position", robot, robot.model().getJointId("wrist_3_joint"), Id, Id, m
)

solver = BySubstitution(robot.configSpace())
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.Equality,
)
solver.add(Implicit.create(pc, cts, [True, True, True]), 0)

print(solver)

# This only tests the call to add.
# The inputs are not valid so the solver
# will not be able to solve anything.
solver.explicitConstraintSet().add(
    Explicit.create(
        robot.configSpace(),
        pc,
        [
            segment(0, 2),
        ],
        [
            segment(2, 4),
        ],
        [
            segment(0, 2),
        ],
        [
            segment(2, 4),
        ],
        cts,
    )
)
solver.explicitConstraintSetHasChanged()

print(solver)
