#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

from hpp import Transform
from pinocchio import SE3


# # This class provides tools to create static stability constraints
class StaticStabilityConstraintsFactory:
    def __init__(self, problem, robot):
        self.problem = problem
        self.robot = robot

    def _getCOM(self, com):
        from numpy import array

        if com == "":
            return array(self.robot.getCenterOfMass())
        else:
            return array(self.problem.getPartialCom(com))

    # # Create static stability constraints where the robot slides on the ground,
    # # \param prefix prefix of the names of the constraint
    # # \param comName name of the PartialCOM. Put "" for
    # #        a full COM computations.
    # # \param leftAnkle, rightAnkle: names of the ankle joints.
    # # \param q0 input configuration for computing constraint reference,
    # # \return a list of the names of the created constraints

    def createSlidingStabilityConstraint(
        self, prefix, comName, leftAnkle, rightAnkle, q0
    ):
        created_constraints = dict()

        _tfs = self.robot.getJointsPosition(q0, [leftAnkle, rightAnkle])
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        self.robot.currentConfiguration(q0)
        x = self._getCOM(comName)
        result = []
        # COM wrt left ankle frame
        xloc = Ml.inverse().transform(x)
        result.append(prefix + "relative-com")
        constraint = self.problem.createRelativeComConstraint(
            result[-1], comName, leftAnkle, xloc, [True] * 3
        )
        created_constraints[result[-1]] = constraint

        result.append(prefix + "relative-pose")
        rel_transform = Mr.inverse() * Ml
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            leftAnkle,
            rightAnkle,
            SE3.Identity(),
            SE3(rel_transform.quaternion.toRotationMatrix(), rel_transform.translation),
            [True] * 6,
        )
        created_constraints[result[-1]] = constraint

        result.append(prefix + "pose-left-foot")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            leftAnkle,
            SE3(Ml.quaternion.toRotationMatrix(), Ml.translation),
            SE3.Identity(),
            [False, False, True, True, True, False],
        )
        created_constraints[result[-1]] = constraint

        result.append(prefix + "pose-left-foot-complement")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            leftAnkle,
            SE3(Ml.quaternion.toRotationMatrix(), Ml.translation),
            SE3.Identity(),
            [True, True, False, False, False, True],
        )
        self.problem.setConstantRightHandSide(constraint, False)
        created_constraints[result[-1]] = constraint

        return created_constraints

    # # # Create static stability constraints where the feet are fixed on the ground,
    def createStaticStabilityConstraint(
        self, prefix, comName, leftAnkle, rightAnkle, q0, maskCom=[True] * 3
    ):
        created_constraints = dict()

        _tfs = self.robot.getJointsPosition(q0, [leftAnkle, rightAnkle])
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        self.robot.currentConfiguration(q0)
        x = self._getCOM(comName)
        result = []

        # COM wrt left ankle frame
        xloc = Ml.inverse().transform(x)
        result.append(prefix + "relative-com")
        constraint = self.problem.createRelativeComConstraint(
            result[-1], comName, leftAnkle, xloc, maskCom
        )
        created_constraints[result[-1]] = constraint

        # Pose of the left foot
        result.append(prefix + "pose-left-foot")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            leftAnkle,
            SE3(Ml.quaternion.toRotationMatrix(), Ml.translation),
            SE3.Identity(),
            [True] * 6,
        )
        created_constraints[result[-1]] = constraint

        # Pose of the right foot
        result.append(prefix + "pose-right-foot")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            rightAnkle,
            SE3(Mr.quaternion.toRotationMatrix(), Mr.translation),
            SE3.Identity(),
            [True] * 6,
        )
        created_constraints[result[-1]] = constraint

        return created_constraints

    def createAlignedCOMStabilityConstraint(
        self, prefix, comName, leftAnkle, rightAnkle, q0, sliding
    ):
        created_constraints = dict()

        _tfs = self.robot.getJointsPosition(q0, [leftAnkle, rightAnkle])
        Ml = Transform(_tfs[0])
        Mr = Transform(_tfs[1])
        self.robot.currentConfiguration(q0)
        x = self._getCOM(comName)
        result = []

        # COM between feet
        result.append(prefix + "com-between-feet")
        # TODO: verify createComBetweenFeet parameters conversion
        constraint = self.problem.createComBetweenFeet(
            result[-1],
            comName,
            leftAnkle,
            rightAnkle,
            [0, 0, 0],
            [0, 0, 0],
            "",
            x,
            [True] * 4,
        )
        created_constraints[result[-1]] = constraint

        if sliding:
            mask = [False, False, True, True, True, False]
        else:
            mask = [True] * 6

        # Pose of the right foot
        result.append(prefix + "pose-right-foot")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            rightAnkle,
            SE3(Mr.quaternion.toRotationMatrix(), Mr.translation),
            SE3.Identity(),
            mask,
        )
        created_constraints[result[-1]] = constraint

        # Pose of the left foot
        result.append(prefix + "pose-left-foot")
        constraint = self.problem.createTransformationConstraint(
            result[-1],
            "",
            leftAnkle,
            SE3(Ml.quaternion.toRotationMatrix(), Ml.translation),
            SE3.Identity(),
            mask,
        )
        created_constraints[result[-1]] = constraint

        return created_constraints
