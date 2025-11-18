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
        jointNames = self.robot.model().names
        for ro in self.robotsAndObjects:
            le = len(ro)
            self.robotToJoints[ro] = [
                n for n in jointNames 
                if n[:le] == ro and (len(n) == le or n[le] in self.separators)
            ]
        self.robotToJoints["universe"] = ["universe"]
        self.jointToRobot = dict()
        for ro, joints in self.robotToJoints.items():
            for j in joints:
                self.jointToRobot[j] = ro

    def computeGrippers(self):
        self.gripperToRobot = dict()
        self.gripperToJoints = dict()
        
        for g in self.factory.grippers:
            for joint_name in self.robot.model().names:
                if g.startswith(joint_name):
                    self.gripperToRobot[g] = self.jointToRobot.get(joint_name, "unknown")
                    self.gripperToJoints[g] = [joint_name]
                    break

    def computePossibleContacts(self):
        self.contactSurfaces = {k: [] for k in self.robotToJoints.keys()}
        self.possibleContacts = [
            (o1, o2) 
            for o1, l1 in self.contactSurfaces.items() 
            for o2, l2 in self.contactSurfaces.items() 
            if o1 != o2 and len(l1) > 0 and len(l2) > 0
        ]

    def setSecurityMarginBetween(self, obj1, obj2, margin):
        self.marginMatrix[frozenset([obj1, obj2])] = margin

    def getSecurityMarginBetween(self, obj1, obj2):
        return self.marginMatrix.get(frozenset([obj1, obj2]), self.defaultMargin)

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
        
        all_constraint_objects = set(constraints_from + constraints_graph + constraints_to)
        
        active_names = []
        if hasattr(factory, 'constraints') and hasattr(factory.constraints, 'available_constraints'):
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
                for o, handle_indices in zip(factory.objects, factory.handlesPerObjects):
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
                                graph.setSecurityMarginForTransition(edge, j1, j2, margin)
            
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
