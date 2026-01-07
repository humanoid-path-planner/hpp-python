#!/usr/bin/env python
#
# Copyright (c) 2020 CNRS, Airbus SAS
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


class SecurityMargins:
    defaultMargin = 0
    separators = ["/"]

    def __init__(self, problem, factory, robotsAndObjects, robot):
        self.problem = problem
        self.robot = robot
        self.factory = factory
        self.robotsAndObjects = robotsAndObjects
        self.marginMatrix = dict()
        self.computeJoints()
        self.computeGrippers()
        self.computePossibleContacts()

    def computeJoints(self):
        self.robotToJoints = dict()
        model = self.robot.model()
        jointNames = list(model.names)
        for ro in self.robotsAndObjects:
            le = len(ro)
            self.robotToJoints[ro] = list(
                filter(
                    lambda n: n[:le] == ro and n[le] in self.separators,
                    jointNames,
                )
            )
        self.robotToJoints["universe"] = ["universe"]
        self.jointToRobot = dict()
        for ro, joints in self.robotToJoints.items():
            for j in joints:
                self.jointToRobot[j] = ro

    def _getChildJoints(self, jointName):
        model = self.robot.model()
        try:
            jointId = model.getJointId(jointName)
        except Exception:
            return []

        childNames = []
        if jointId < len(model.children):
            for childId in model.children[jointId]:
                if childId < len(model.names):
                    childNames.append(model.names[childId])
        return childNames

    def _getGripperJoint(self, gripperName):
        model = self.robot.model()
        if model.existFrame(gripperName):
            frameId = model.getFrameId(gripperName)
            frame = model.frames[frameId]
            parentJointId = frame.parentJoint
            if parentJointId < len(model.names):
                return model.names[parentJointId]
        return None

    def computeGrippers(self):
        self.gripperToRobot = dict()
        self.gripperToJoints = dict()
        for g in self.factory.grippers:
            j = self._getGripperJoint(g)
            if j is not None and j in self.jointToRobot:
                self.gripperToRobot[g] = self.jointToRobot[j]
                childJoints = self._getChildJoints(j)
                self.gripperToJoints[g] = [j] + childJoints
            else:
                for joint_name in self.robot.model().names:
                    if g.startswith(joint_name) and joint_name in self.jointToRobot:
                        self.gripperToRobot[g] = self.jointToRobot[joint_name]
                        childJoints = self._getChildJoints(joint_name)
                        self.gripperToJoints[g] = [joint_name] + childJoints
                        break

    def computePossibleContacts(self):
        self.contactSurfaces = dict()
        for k in self.robotToJoints.keys():
            self.contactSurfaces[k] = list()

        for io, obj_name in enumerate(self.factory.objects):
            if io < len(self.factory.contactsPerObjects):
                for surface_name in self.factory.contactsPerObjects[io]:
                    if obj_name in self.contactSurfaces:
                        self.contactSurfaces[obj_name].append(surface_name)

        for surface_name in self.factory.envContacts:
            found = False
            for ro in list(self.robotsAndObjects) + ["universe"]:
                for sep in self.separators:
                    prefix = ro + sep
                    if surface_name.startswith(prefix):
                        if ro in self.contactSurfaces:
                            self.contactSurfaces[ro].append(surface_name)
                        found = True
                        break
                if found:
                    break
            if not found:
                self.contactSurfaces["universe"].append(surface_name)

        self.possibleContacts = list()
        for o1, l1 in self.contactSurfaces.items():
            for o2, l2 in self.contactSurfaces.items():
                if o1 != o2 and len(l1) > 0 and len(l2) > 0:
                    self.possibleContacts.append((o1, o2))

    def setSecurityMarginBetween(self, obj1, obj2, margin):
        self.marginMatrix[frozenset([obj1, obj2])] = margin

    def getSecurityMarginBetween(self, obj1, obj2):
        key = frozenset([obj1, obj2])
        return self.marginMatrix.get(key, self.defaultMargin)

    def getActiveConstraintsAlongEdge(self, edge):
        factory = self.factory
        graph = factory.graph

        result = graph.getNodesConnectedByTransition(edge)
        from_name, to_name = result if isinstance(result, tuple) else (result, result)

        state_from = graph.getState(from_name)
        state_to = graph.getState(to_name)

        constraints_from = graph.getNumericalConstraintsForState(state_from)
        constraints_graph = graph.getNumericalConstraintsForGraph()
        constraints_to = graph.getNumericalConstraintsForState(state_to)

        all_constraint_objects = set(
            constraints_from + constraints_graph + constraints_to
        )

        active_names = []
        if hasattr(factory, "constraints") and hasattr(
            factory.constraints, "available_constraints"
        ):
            constraint_dict = factory.constraints.available_constraints
            for name, constraint_obj in constraint_dict.items():
                if constraint_obj in all_constraint_objects:
                    active_names.append(name)

        res = {"place": [], "grasp": []}

        for c in active_names:
            for o in factory.objects:
                if c == "place_" + o:
                    res["place"].append(o)
            for g in factory.grippers:
                for o, handle_indices in zip(
                    factory.objects, factory.handlesPerObjects
                ):
                    for h_idx in handle_indices:
                        h = factory.handles[h_idx]
                        if c == g + " grasps " + h:
                            res["grasp"].append((g, o))

        return res

    def apply(self):
        factory = self.factory
        graph = factory.graph
        edges = graph.getTransitions()

        for edge in edges:
            for i1, ro1 in enumerate([*self.robotsAndObjects, "universe"]):
                for i2, ro2 in enumerate([*self.robotsAndObjects, "universe"]):
                    if i2 < i1:
                        continue
                    margin = self.getSecurityMarginBetween(ro1, ro2)
                    for j1 in self.robotToJoints[ro1]:
                        for j2 in self.robotToJoints[ro2]:
                            if j1 != j2:
                                graph.setSecurityMarginForTransition(
                                    edge, j1, j2, margin
                                )

            constraints = self.getActiveConstraintsAlongEdge(edge)

            for g, ro1 in constraints["grasp"]:
                if g in self.gripperToJoints and ro1 in self.robotToJoints:
                    for j1 in self.robotToJoints[ro1]:
                        for j2 in self.gripperToJoints[g]:
                            graph.setSecurityMarginForTransition(edge, j1, j2, 0)

            for o1 in constraints["place"]:
                for o2, o3 in self.possibleContacts:
                    if o1 == o2:
                        for j1 in self.robotToJoints[o1]:
                            for j2 in self.robotToJoints[o3]:
                                graph.setSecurityMarginForTransition(edge, j1, j2, 0)
