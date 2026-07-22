#!/usr/bin/env python3
#
# Copyright (c) 2025 CNRS
# Author: Florent Lamiraux
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


def shrinkJointRange(robot, joints, ratio):
    """
    Reduce the range of selected joints for security

      Input
        - robot: an instance of Robot class,
        - joints the list of joint names,
        - ratio: the range of the joint is shrunk around the middle of its
                 bounds by this ratio.
    """
    model = robot.model()
    for j in joints:
        rank = model.getJointId(j)
        iq = model.joints[rank].idx_q
        assert model.joints[rank].nq == 1
        bounds = [model.lowerPositionLimit[iq], model.upperPositionLimit[iq]]
        width = bounds[1] - bounds[0]
        if width < 0:
            raise ValueError(
                "Cannot shrink range of joint " + j + ". The joint is not bounded."
            )
        mean = 0.5 * (bounds[1] + bounds[0])
        m = mean - 0.5 * ratio * width
        M = mean + 0.5 * ratio * width
        model.lowerPositionLimit[iq] = m
        model.upperPositionLimit[iq] = M

def projectInJointRange(robot, q, epsilon):
    """
    Project a configuration into the joint bounds of a robot

      Input
        - robot:   an instance of Robot class,
        - q:       input configuration
        - epsilon: distance inside the joint bounds each configuration variable is projected to
                   in order to avoid projecting on the joint limits
    """
    model = robot.model()
    result = q.copy()
    for i in range(model.njoints):
        if model.joints[i].nq != 1:
            continue
        iq = model.joints[i].idx_q
        m, M = [model.lowerPositionLimit[iq], model.upperPositionLimit[iq]]
        if m < q[iq] and q[iq] < M:
            continue
        if M - m  < 2*epsilon:
            result[iq] = .5*(m+M)
        else:
            if q[iq] > M:
                result[iq] = M - epsilon
            elif q[iq] < m:
                result[iq] = m + epsilon
    return result
