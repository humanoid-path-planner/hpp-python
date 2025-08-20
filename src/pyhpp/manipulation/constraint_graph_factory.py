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

import abc
import re
import sys
import numpy as np
from pyhpp.constraints import Implicit, LockedJoint


class Constraints:
    """
    Container of numerical constraints

    Numerical constraints are stored as
    \\li grasp,
    \\li pregrasp, or
    \\li numerical constraint,
    """

    def __init__(self, grasps=[], pregrasps=[], numConstraints=[], lockedJoints=[]):
        if isinstance(grasps, str):
            raise TypeError("argument grasps should be a list of strings")
        if isinstance(pregrasps, str):
            raise TypeError("argument pregrasps should be a list of strings")
        if isinstance(numConstraints, str):
            raise TypeError("argument numConstraints should be a list of strings")
        if lockedJoints != []:
            from warnings import warn

            warn(
                "argument lockedJoints in constructor of class "
                + "hpp.corbaserver.manipulation.constraints.Constraints "
                + "is deprecated. Locked joints are handled as numerical "
                + "constraints."
            )
            numConstraints.extend(lockedJoints)
        self._grasps = set(grasps)
        self._pregrasps = set(pregrasps)
        self._numConstraints = set(numConstraints)

    def __add__(self, other):
        res = Constraints(
            grasps=self._grasps | other._grasps,
            pregrasps=self._pregrasps | other._pregrasps,
            numConstraints=self._numConstraints | other._numConstraints,
        )
        return res

    def __sub__(self, other):
        res = Constraints(
            grasps=self._grasps - other._grasps,
            pregrasps=self._pregrasps - other._pregrasps,
            numConstraints=self._numConstraints - other._numConstraints,
        )
        return res

    def __iadd__(self, other):
        self._grasps |= other._grasps
        self._pregrasps |= other._pregrasps
        self._numConstraints |= other._numConstraints
        return self

    def __isub__(self, other):
        self._grasps -= other._grasps
        self._pregrasps -= other._pregrasps
        self._numConstraints -= other._numConstraints
        return self

    def empty(self):
        for s in [self._grasps, self._pregrasps, self._numConstraints]:
            if len(s) > 0:
                return False
        return True

    @property
    def grasps(self):
        return list(self._grasps)

    @property
    def pregrasps(self):
        return list(self._pregrasps)

    @property
    def numConstraints(self):
        return list(self._numConstraints)

    def __str__(self):
        res = "constraints\n"
        res += "  grasps: "
        for c in self._grasps:
            res += c + ", "
        res += "\n  pregrasps: "
        for c in self._pregrasps:
            res += c + ", "
        res += "\n  numConstraints: "
        for c in self._numConstraints:
            res += c + ", "
        return res


class PossibleGrasps:
    def __init__(self, grippers, handles, grasps):
        """
        Constructor
        param grasps a dictionaty whose keys are the grippers registered in the
               factory and whose values are lists of handles also registered in
               the factory
        """
        self.possibleGrasps = list()
        for ig, gripper in enumerate(grippers):
            handles_ = grasps.get(gripper, list())
            handleIndices = list()
            handleIndices = list(map(handles.index, handles_))
            self.possibleGrasps.append(handleIndices)

    def __call__(self, grasps):
        for ig, ih in enumerate(grasps):
            if ih is not None and ih not in self.possibleGrasps[ig]:
                return False
        return True


class GraspIsAllowed:
    """Class that stores grasp validation instances"""

    def __init__(self):
        """
        Successively calls all the validation instances
         param grasps the set of grasp to validate
         return False if one validation fails, True otherwise
        """
        self.graspValidations_ = list()

    def __call__(self, grasps):
        for gv in self.graspValidations_:
            if not gv(grasps):
                return False
        return True

    def append(self, graspValidation):
        self.graspValidations_.append(graspValidation)


class Rules:
    def __init__(self, grippers, handles, rules):
        rs = []
        status = []
        for r in rules:
            # replace empty strings by the corresponding regexp "^$",
            # otherwise "" matches with all strings.
            for i in range(len(r.grippers)):
                if r.grippers[i] == "":
                    r.grippers[i] = "^$"
            for i in range(len(r.handles)):
                if r.handles[i] == "":
                    r.handles[i] = "^$"
            handlesRegex = [None] * len(grippers)
            for j, gr in enumerate(r.grippers):
                grc = re.compile(gr)
                for i, g in enumerate(grippers):
                    if grc.match(g):
                        assert handlesRegex[i] is None
                        handlesRegex[i] = re.compile(r.handles[j])
            status.append(r.link)

            rs.append(tuple(handlesRegex))
        self.rules = tuple(rs)
        self.status = tuple(status)
        self.handles = tuple(handles)
        self.defaultAcceptation = False

    def __call__(self, grasps):
        for r, s in zip(self.rules, self.status):
            apply = True
            for i, h in enumerate(r):
                if h is not None and not h.match(
                    "" if grasps[i] is None else self.handles[grasps[i]]
                ):
                    # This rule does not apply
                    apply = False
                    break
            if apply:
                return s
        return self.defaultAcceptation


if sys.version_info.major == 2:

    class ABC:
        """Python 2.7 equivalent to abc.ABC Python 3 class."""

        __metaclass__ = abc.ABCMeta

else:
    from abc import ABC


class GraphFactoryAbstract(ABC):
    """
    An abstract class which is loops over the different (gripper, handle) associations.

    The behaviour can be tuned by setting the callback functions:
    - \\ref graspIsAllowed (redundant with \\ref setRules)
    - \\ref constraint_graph_factory_algo_callbacks "Algorithm steps"

    <b>Sketch of the algorithm</b>

    Let
     \\li \\f$G\\f$ be the set of grippers,
     \\li \\f$H\\f$ be the set of handles.

     A \b grasp is defined as a pair \\f$(g,h)\\in G\\times H\\f$.

     Each \b state (excluding waypoint states) is defined by a set of grasps.
     Given a set of grasps, we define
     \\li <b>available grippers</b> as the grippers that are not the first
         element of any pair of the set of grasps,
     \\li <b>available handles</b> as the handles that are not the second element
        of any pair of the set of grasps.

     The first node is defined by the empty set. The graph is built recursiveley
     as follows:

     if the set of grasps defining the node is not allowed (method \\link
         constraint_graph_factory.GraphFactoryAbstract.graspIsAllowed
         graspIsAllowed \\endlink), return.

     Otherwise, for any pair \\f$(g,h)\f$ of available grippers and available
     handles,
     \\li build a new state by adding grasp \\f$(g,h)\\f$ to the current set of
         grasps (method
         \\link constraint_graph_factory.GraphFactoryAbstract.makeState
         makeState\\endlink),
     \\li build a transition from the current state to the new state (method \\link
         constraint_graph_factory.GraphFactoryAbstract.makeTransition
         makeTransition \\endlink)
     \\li build a loopTransition from the current state to itself (method \\link
         constraint_graph_factory.GraphFactoryAbstract.makeLoopTransition
         makeLoopTransition \\endlink)
     \\li repeat the two above states to the new state.
    """

    def __init__(self):
        # # Reduces the problem combinatorial.
        # Function called to check whether a grasps is allowed.
        # It takes as input a list of handle indices (or None) such
        # that i-th \\ref grippers grasps `grasps[i]`-th \\ref handles.
        # It must return a boolean
        #
        # It defaults to: \code lambda x : True
        self.graspIsAllowed = GraspIsAllowed()

        # # \name Internal variables
        # \{

        self.states = dict()
        self.transitions = set()
        # # the handle names
        self.handles = tuple()  # strings
        # # the gripper names
        self.grippers = tuple()  # strings
        # # the names of contact on the environment
        self.envContacts = tuple()  # strings
        # # the object names
        self.objects = tuple()  # strings
        # # See \\ref setObjects
        self.handlesPerObjects = tuple()  # object index to handle indixes
        # # See \\ref setObjects
        self.objectFromHandle = tuple()  # handle index to object index
        # # See \\ref setObjects
        self.contactsPerObjects = tuple()  # object index to contact names
        # # \}

    # # \name Main API
    # \{

    def setGrippers(self, grippers):
        """
        \\param grippers list of gripper names to be considered
        """
        assert isinstance(grippers, (list, tuple))
        self.grippers = tuple(grippers)

    def setObjects(self, objects, handlesPerObjects, contactsPerObjects):
        """
        \\param objects list of object names to be considered
        \\param handlesPerObjects a list of list of handle names.
        \\param contactsPerObjects a list of list of contact names.
         handlesPerObjects and contactsPerObjects must have one list for each object,
         in the same order.
        """
        if len(objects) != len(handlesPerObjects):
            raise IndexError("There should be as many handlesPerObjects as objects")
        if len(objects) != len(contactsPerObjects):
            raise IndexError("There should be as many contactsPerObjects as objects")
        self.objects = tuple(objects)
        handles = []
        hpo = []
        cpo = []
        ofh = []
        for io, o in enumerate(self.objects):
            hpo.append(
                tuple(range(len(handles), len(handles) + len(handlesPerObjects[io])))
            )
            handles.extend(handlesPerObjects[io])
            ofh.extend([io] * len(handlesPerObjects[io]))
            cpo.append(tuple(contactsPerObjects[io]))

        self.handles = tuple(handles)
        self.handlesPerObjects = tuple(hpo)
        self.objectFromHandle = tuple(ofh)
        self.contactsPerObjects = tuple(cpo)

    def environmentContacts(self, envContacts):
        """
        \\param envContacts contact on the environment to be considered.
        """
        self.envContacts = tuple(envContacts)

    def setRules(self, rules):
        """
        Set the function \\ref graspIsAllowed
        \\param rules a list of Rule objects
        """
        self.graspIsAllowed.append(Rules(self.grippers, self.handles, rules))

    def setPossibleGrasps(self, grasps):
        """
        Define the possible grasps
        \\param grasps a dictionaty whose keys are the grippers registered in
                the factory and whose values are lists of handles also
                registered in the factory
        """
        self.graspIsAllowed.append(PossibleGrasps(self.grippers, self.handles, grasps))

    def generate(self):
        """
        Go through the combinatorial defined by the grippers and handles
        and create the states and transitions.
        """
        grasps = (None,) * len(self.grippers)
        self._recurse(self.grippers, self.handles, grasps, 0)

    # # \}

    # # \name Abstract methods of the algorithm
    #  \anchor constraint_graph_factory_algo_callbacks
    # \{

    @abc.abstractmethod
    def makeState(self, grasps, priority):
        """
        Create a new state.
        \\param grasps a handle index for each gripper,
                as in GraphFactoryAbstract.graspIsAllowed.
        \\param priority the state priority.
        \\return an object representing the state.
        """
        return grasps

    @abc.abstractmethod
    def makeLoopTransition(self, state):
        """
        Create a loop transition.
        \\param state: an object returned by \\ref makeState which represent the state
        """
        pass

    def transitionIsAllowed(self, stateFrom, stateTo):
        """
        Check whether a transition between two states is allowed
        \\param stateFrom, stateTo states to connect
        """
        return True

    @abc.abstractmethod
    def makeTransition(self, stateFrom, stateTo, ig):
        """
        Create two transitions between two different states.
        \\param stateFrom: same as grasps in \\ref makeState
        \\param stateTo: same as grasps in \\ref makeState
        \\param ig: index if the grasp vector that changes, i.e. such that
          - \\f$ stateFrom.grasps[i_g] \neq stateTo.grasps[i_g] \\f$
          - \\f$ \\forall i \neq i_g, stateFrom.grasps[i] = stateTo.grasps[i] \\f$
        """
        pass

    # # \}

    def _existState(self, grasps):
        return grasps in self.states

    def _makeState(self, grasps, priority):
        if not self._existState(grasps):
            state = self.makeState(grasps, priority)
            self.states[grasps] = state

            # Create loop transition
            self.makeLoopTransition(state)
        else:
            state = self.states[grasps]
        return state

    def _isObjectGrasped(self, grasps, object):
        for h in self.handlesPerObjects[object]:
            if h in grasps:
                return True
        return False

    def _stateName(self, grasps, abbrev=False):
        sepGH = "-" if abbrev else " grasps "
        sep = ":" if abbrev else " : "
        name = sep.join(
            [
                (str(ig) if abbrev else self.grippers[ig])
                + sepGH
                + (str(ih) if abbrev else self.handles[ih])
                for ig, ih in enumerate(grasps)
                if ih is not None
            ]
        )
        if len(name) == 0:
            return "f" if abbrev else "free"
        return name

    def _transitionNames(self, sFrom, sTo, ig):
        g = self.grippers[ig]
        h = self.handles[sTo.grasps[ig]]
        sep = " | "
        return (
            g + " > " + h + sep + self._stateName(sFrom.grasps, True),
            g + " < " + h + sep + self._stateName(sTo.grasps, True),
        )

    def _loopTransitionName(self, grasps):
        return "Loop | " + self._stateName(grasps, True)

    def _recurse(self, grippers, handles, grasps, depth):
        """
        Recurse across all possible sets of grasps

        This method visits all possible set of grasps and create states
        and transitions between those states.

        \\param grippers list of names of available grippers: that do not hold
               an handle yet,
        \\param handles list of names of available handles that are not hold yet
               by a gripper.
        \\param grasps list of grasps already active. Grasps are represented by
               a list of handle indices or None if the gripper is available.
               the order in the list corresponds to the order of the gripper
               in the list of all grippers.
               For instance, if a robot has 3 grippers registered in the factory
               ("g1", "g2", "g3"), handles ["h1", "h2"] are registered in
               the factory, and grasps is equal to (1, 0, None), then
                 \\li "g1" holds "h2",
                 \\li "g2" holds "h1", and
                 \\li "g3" does not hold anything.

        """
        isAllowed = self.graspIsAllowed(grasps)
        if isAllowed:
            current = self._makeState(grasps, depth)

        if len(grippers) == 0 or len(handles) == 0:
            return
        for ig, g in enumerate(grippers):
            # ngrippers = { grippers } \ grippers[ig]
            ngrippers = grippers[:ig] + grippers[ig + 1 :]
            # isg <- index of g in list of all grippers
            isg = self.grippers.index(g)
            for ih, h in enumerate(handles):
                # nhandles = { handles } \ handles[ih]
                nhandles = handles[:ih] + handles[ih + 1 :]
                # ish <- index of h in all handles
                ish = self.handles.index(h)
                # nGrasp <- substitute current handle index at current gripper
                # position.
                nGrasps = grasps[:isg] + (ish,) + grasps[isg + 1 :]

                nextIsAllowed = self.graspIsAllowed(nGrasps)
                isNewState = not self._existState(nGrasps)
                if nextIsAllowed:
                    nnext = self._makeState(nGrasps, depth + 1)

                if (
                    isAllowed
                    and nextIsAllowed
                    and self.transitionIsAllowed(stateFrom=current, stateTo=nnext)
                ):
                    self.makeTransition(current, nnext, isg)

                if isNewState:
                    self._recurse(ngrippers, nhandles, nGrasps, depth + 2)


class ConstraintFactoryAbstract(ABC):
    """
    An abstract class which stores the constraints.

    Child classes are responsible for building them.
    - \\ref buildGrasp
    - \\ref buildPlacement
    """

    def __init__(self, graphfactory):
        self._grasp = dict()
        self._placement = dict()

        self.graphfactory = graphfactory

    # # \name Accessors to the different elementary constraints
    # \{

    def getGrasp(self, gripper, handle):
        """
        Get constraints relative to a grasp

        \\param gripper name of a gripper or gripper index
        \\param handle name of a handle or handle index
        \\return a dictionary with keys <c>['grasp', 'graspComplement',
                'preGrasp']</c> and values the corresponding constraints as
                \\link manipulation.constraints.Constraints Constraints\\endlink
                instances.
        \\warning If grasp does not exist, the function creates it.
        """
        if isinstance(gripper, str):
            ig = self.graphfactory.grippers.index(gripper)
        else:
            ig = gripper
        if isinstance(handle, str):
            ih = self.graphfactory.handles.index(handle)
        else:
            ih = handle
        k = (ig, ih)
        if k not in self._grasp:
            self._grasp[k] = self.buildGrasp(
                self.graphfactory.grippers[ig],
                None if ih is None else self.graphfactory.handles[ih],
            )
            assert isinstance(self._grasp[k], dict)
        return self._grasp[k]

    def g(self, gripper, handle, what):
        """
        Get constraints relative to a grasp

        \\param gripper name of a gripper or gripper index,
        \\param handle name of a handle or handle index,
        \\param what a word among <c>['grasp', 'graspComplement', 'preGrasp']</c>.
        \\return the corresponding constraints as
                \\link manipulation.constraints.Constraints Constraints\\endlink
                instances.
        \\warning If grasp does not exist, the function creates it.
        """
        return self.getGrasp(gripper, handle)[what]

    def getPlacement(self, object):
        """
        Get constraints relative to an object placement

        \\param object object name or index
        \\return a dictionary with keys <c>['placement', 'placementComplement',
                'prePlacement']</c> and values the corresponding constraints as
                \\link manipulation.constraints.Constraints Constraints\\endlink
                instances.
        \\warning If placement does not exist, the function creates it.
        """
        if isinstance(object, str):
            io = self.graphfactory.objects.index(object)
        else:
            io = object
        k = io
        if k not in self._placement:
            self._placement[k] = self.buildPlacement(self.graphfactory.objects[io])
        return self._placement[k]

    def p(self, object, what):
        """
        Get constraints relative to a placement

        \\param object object name or index
        \\param what a word among <c>['placement', 'placementComplement',
               'prePlacement']</c>.
        \\return the corresponding constraints as
                \\link manipulation.constraints.Constraints Constraints\\endlink
                instances.
        \\warning If placement does not exist, the function creates it.
        """
        return self.getPlacement(object)[what]

    # # \}

    @abc.abstractmethod
    def buildGrasp(self, g, h):
        """
        Function called to create grasp constraints.
        Must return a tuple of Constraints objects as:
        - constraint that validates the grasp
        - constraint that parameterizes the graph
        - constraint that validates the pre-grasp
        \\param g gripper string
        \\param h handle  string
        """
        return (
            None,
            None,
            None,
        )

    @abc.abstractmethod
    def buildPlacement(self, o):
        """
        Function called to create placement constraints.
        Must return a tuple of Constraints objects as:
        - constraint that validates placement
        - constraint that parameterizes placement
        - constraint that validates pre-placement
        \\param o string
        """
        return (
            None,
            None,
            None,
        )


class ConstraintFactory(ConstraintFactoryAbstract):
    """
    Default implementation of ConstraintFactoryAbstract
    """

    removeEmptyConstraints = True
    """
    This flag determines whether preplacement states are added when no
    contact surface is present. Setting it to false may be useful in order
    to keep the same graph topology and to populate the graph with dedicated
    constraints.
    """

    def _removeEmptyConstraints(self, constraints):
        if self.removeEmptyConstraints:
            return [
                n
                for n in constraints
                if n in self.available_constraints
                and Implicit.getFunctionOutputSize(self.available_constraints[n]) > 0
            ]
        else:
            return constraints

    gfields = ("grasp", "graspComplement", "preGrasp")
    pfields = ("placement", "placementComplement", "prePlacement")

    def __init__(self, graphfactory, graph, constraints=dict()):
        super().__init__(graphfactory)
        self.graph = graph
        self.available_constraints = dict()
        self.registerConstraints(constraints)

    def registerConstraint(self, constraint, constraint_name):
        """Register a single constraint as available"""
        self.available_constraints[constraint_name] = constraint

    def registerConstraints(self, constraint_dict):
        """Register multiple constraints as available"""
        self.available_constraints.update(constraint_dict)

    def buildGrasp(self, g, h):
        """
        Calls ConstraintGraph.createGraph and ConstraintGraph.createPreGrasp
        \\param g gripper string
        \\param h handle  string
        \note if the grasp constraint already exists, it is not created.
        """
        n = g + " grasps " + h
        pn = g + " pregrasps " + h
        graspAlreadyCreated = n in self.available_constraints
        pregraspAlreadyCreated = pn in self.available_constraints
        if not graspAlreadyCreated:
            cname = n + "/complement"
            bname = n + "/hold"
            gripper = self.graph.robot.grippers()[g]
            handle = self.graph.robot.handles()[h]
            constraint = handle.createGrasp(gripper, n)
            complement = handle.createGraspComplement(gripper, cname)
            both = handle.createGraspAndComplement(gripper, bname)
            self.registerConstraint(constraint, n)
            self.registerConstraint(complement, cname)
            self.registerConstraint(both, bname)
        if not pregraspAlreadyCreated:
            gripper = self.graph.robot.grippers()[g]
            handle = self.graph.robot.handles()[h]
            c = handle.clearance + gripper.clearance
            pregrasp = handle.createPreGrasp(gripper, c, pn)
            self.registerConstraint(pregrasp, pn)
        return dict(
            list(
                zip(
                    self.gfields,
                    (
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    n,
                                ],
                            )
                        ),
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    n + "/complement",
                                ],
                            )
                        ),
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    pn,
                                ],
                            )
                        ),
                    ),
                )
            )
        )

    def buildPlacement(self, o):
        """
        This implements placement manifolds,
        where the parameterization constraints is the complement
        of the placement constraint.
        \\param o string
        """
        n = "place_" + o
        pn = "preplace_" + o
        # Get distance of object to surface in preplacement
        distance = self.graphfactory.getPreplacementDistance(o)
        io = self.graphfactory.objects.index(o)
        placeAlreadyCreated = n in self.available_constraints
        if (
            len(self.graphfactory.contactsPerObjects[io]) == 0
            or len(self.graphfactory.envContacts) == 0
        ) and not placeAlreadyCreated:
            ljs = []
            for n in self.graph.robot.getJointNames():
                if n.startswith(o + "/"):
                    ljs.append(n)
                    q = self.graph.robot.getJointConfig(n)
                    self.registerConstraint(
                        LockedJoint.create(
                            self.graph.robot.asPinDevice(), n, np.array(q)
                        ),
                        n,
                    )
            return dict(
                list(
                    zip(
                        self.pfields,
                        (
                            Constraints(),
                            Constraints(numConstraints=ljs),
                            Constraints(),
                        ),
                    )
                )
            )
        if not placeAlreadyCreated:
            constraint, complement, both = self.graph.createPlacementConstraint(
                n,
                list(self.graphfactory.contactsPerObjects[io]),
                list(self.graphfactory.envContacts),
            )
            self.registerConstraint(constraint, n)
            self.registerConstraint(complement, n + "/complement")
            if both:
                self.registerConstraint(both, n + "/hold")
        if pn not in self.available_constraints:
            preplace_constraint = self.graph.createPrePlacementConstraint(
                pn,
                list(self.graphfactory.contactsPerObjects[io]),
                list(self.graphfactory.envContacts),
                distance,
            )
            self.registerConstraint(preplace_constraint, pn)
        return dict(
            list(
                zip(
                    self.pfields,
                    (
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    n,
                                ],
                            )
                        ),
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    n + "/complement",
                                ],
                            )
                        ),
                        Constraints(
                            numConstraints=self._removeEmptyConstraints(
                                [
                                    pn,
                                ],
                            )
                        ),
                    ),
                )
            )
        )


class ConstraintGraphFactory(GraphFactoryAbstract):
    """
    Default implementation of ConstraintGraphFactory

    The minimal usage is the following:
    >>> graph = ConstraintGraph(robot, "graph")

    # Required calls
    >>> factory = ConstraintGraphFactory(graph)
    >>> factory.setGrippers(["gripper1", ... ])
    >>> factory.setObjects(["object1", ], [ [ "object1/handle1", ... ] ], [ [] ])

    # Optionally
    >>> factory.environmentContacts(["contact1", ... ])
    >>> factory.setRules([ Rule(["gripper1", ..], ["handle1", ...], True), ... ])

    >>> factory.generate()

    # graph is initialized
    """

    class StateAndManifold:
        def __init__(self, factory, grasps, state_obj, name):
            self.grasps = grasps
            self.state_obj = state_obj
            self.name = name
            self.manifold = Constraints()
            self.foliation = Constraints()
            # Add the grasps
            for ig, ih in enumerate(grasps):
                if ih is not None:
                    self.manifold += factory.constraints.g(ig, ih, "grasp")
                    self.foliation += factory.constraints.g(ig, ih, "graspComplement")
            # Add the placement constraints
            for io, object in enumerate(factory.objects):
                if not factory._isObjectGrasped(grasps, io):
                    self.manifold += factory.constraints.p(object, "placement")
                    self.foliation += factory.constraints.p(
                        object, "placementComplement"
                    )

    # default distance between object and surface in preplacement configuration
    defaultPreplaceDist = 0.05
    # See methods setPreplacementDistance and getPreplacementDistance
    preplaceDistance = dict()

    def __init__(self, graph, constraints=dict()):
        """
        \\param graph an instance of ConstraintGraph
        """
        super().__init__()

        # Stores the constraints in a child class of ConstraintFactoryAbstract
        self.constraints = ConstraintFactory(self, graph, constraints)

        self.graph = graph

        # whether there should be placement complement in transition from
        # intersec to preplace
        self.preplaceGuide = False
        self.state_objects = {}
        self.edge_objects = {}

    # # \name Default functions
    # \{

    def makeState(self, grasps, priority):
        # Create state
        name = self._stateName(grasps)
        state_obj = self.graph.createState(name, False, priority)
        state = ConstraintGraphFactory.StateAndManifold(self, grasps, state_obj, name)

        # Add the constraints
        self._add_constraints_to_state(state_obj, state.manifold)
        return state

    def makeLoopTransition(self, state):
        n = self._loopTransitionName(state.grasps)
        loop_edge = self.graph.createTransition(
            state.state_obj, state.state_obj, n, 0, state.state_obj
        )
        self.edge_objects[n] = loop_edge
        self._add_constraints_to_transition(loop_edge, state.foliation)

    def makeTransition(self, stateFrom, stateTo, ig):
        """
        Make transition between two states using boost python API

        \\param stateFrom initial state,
        \\param stateTo new state
        \\param ig index of gripper

        stateTo grasps are the union of stateFrom grasps with a set containing
        a grasp by gripper of index ig in list <c>self.grippers</c> of a handle.
        The index of the newly grasped handle can be found by
        <c>stateTo.grasps[ig]</c>
        """
        sf = stateFrom
        st = stateTo
        names = self._transitionNames(sf, st, ig)
        if names in self.transitions:
            return

        ih = st.grasps[ig]
        iobj = self.objectFromHandle[ih]
        noPlace = self._isObjectGrasped(sf.grasps, iobj)

        gc = self.constraints.g(ig, ih, "grasp")
        gcc = self.constraints.g(ig, ih, "graspComplement")
        pgc = self.constraints.g(ig, ih, "preGrasp")

        if noPlace:
            pc = Constraints()
            pcc = Constraints()
            ppc = Constraints()
        else:
            pc = self.constraints.p(self.objectFromHandle[ih], "placement")
            pcc = self.constraints.p(self.objectFromHandle[ih], "placementComplement")
            ppc = self.constraints.p(self.objectFromHandle[ih], "prePlacement")

        manifold = sf.manifold - pc

        # The different cases:
        pregrasp = not pgc.empty()
        intersec = (not gc.empty()) and (not pc.empty())
        preplace = not ppc.empty()

        nWaypoints = pregrasp + intersec + preplace
        nTransitions = 1 + nWaypoints

        # Check whether level set edge is required
        assert len(sf.foliation.grasps) == 0
        assert len(sf.foliation.pregrasps) == 0
        assert len(st.foliation.grasps) == 0
        assert len(st.foliation.pregrasps) == 0
        crossedFoliation = False
        if (
            len(sf.foliation.numConstraints) > 0
            and len(st.foliation.numConstraints) > 0
        ):
            crossedFoliation = True

        def _createWaypointState(name, constraints):
            """Create waypoint state using boost python API"""
            waypoint_state = self.graph.createState(name, True, 0)
            self.state_objects[name] = waypoint_state
            self._add_constraints_to_state(waypoint_state, constraints)
            return name

        wStates = [sf.name]
        wStateObjects = [sf.state_obj]

        if pregrasp:
            pregrasp_name = names[0] + "_pregrasp"
            _createWaypointState(pregrasp_name, pc + pgc + manifold)
            wStates.append(pregrasp_name)
            wStateObjects.append(self.state_objects[pregrasp_name])

        if intersec:
            intersec_name = names[0] + "_intersec"
            _createWaypointState(intersec_name, pc + gc + manifold)
            wStates.append(intersec_name)
            wStateObjects.append(self.state_objects[intersec_name])

        if preplace:
            preplace_name = names[0] + "_preplace"
            _createWaypointState(preplace_name, ppc + gc + manifold)
            wStates.append(preplace_name)
            wStateObjects.append(self.state_objects[preplace_name])

        wStates.append(st.name)
        wStateObjects.append(st.state_obj)

        if nWaypoints > 0:
            forward_edge = self.graph.createWaypointTransition(
                sf.state_obj, st.state_obj, names[0], nWaypoints, 1, sf.state_obj, False
            )
            backward_edge = self.graph.createWaypointTransition(
                st.state_obj, sf.state_obj, names[1], nWaypoints, 1, sf.state_obj, False
            )

            self.edge_objects[names[0]] = forward_edge
            self.edge_objects[names[1]] = backward_edge

            forward_ls_edge = None
            backward_ls_edge = None

            if crossedFoliation:
                forward_ls_edge = self.graph.createWaypointTransition(
                    sf.state_obj,
                    st.state_obj,
                    names[0] + "_ls",
                    nWaypoints,
                    10,
                    sf.state_obj,
                    False,
                )
                self.edge_objects[names[0] + "_ls"] = forward_ls_edge

                if not noPlace:
                    backward_ls_edge = self.graph.createWaypointTransition(
                        st.state_obj,
                        sf.state_obj,
                        names[1] + "_ls",
                        nWaypoints,
                        10,
                        sf.state_obj,
                        False,
                    )
                    self.edge_objects[names[1] + "_ls"] = backward_ls_edge

            wTransitions = []
            wTransitionObjects = []

            for i in range(nTransitions):
                nf = f"{names[0]}_{i}{i + 1}"
                nb = f"{names[1]}_{i + 1}{i}"

                forward_trans = self.graph.createTransition(
                    wStateObjects[i], wStateObjects[i + 1], nf, -1, wStateObjects[i]
                )
                backward_trans = self.graph.createTransition(
                    wStateObjects[i + 1], wStateObjects[i], nb, -1, wStateObjects[i]
                )
                self.edge_objects[nf] = forward_trans
                self.edge_objects[nb] = backward_trans

                nf_ls = nf
                nb_ls = nb

                if crossedFoliation:
                    if i == 0:
                        edgeName = nf_ls = nf + "_ls"
                        containingState = (
                            sf.state_obj
                        )  # containing state is always start state
                        level_set_edge_forward = self.graph.createLevelSetTransition(
                            wStateObjects[i],
                            wStateObjects[i + 1],
                            edgeName,
                            -1,
                            containingState,
                        )
                        self.edge_objects[edgeName] = level_set_edge_forward

                        paramNC_constraints = self._get_constraint_objects(
                            (st.foliation - sf.foliation).numConstraints
                        )
                        condNC_constraints = self._get_constraint_objects(
                            (st.manifold - sf.manifold).numConstraints
                        )

                        self.graph.addLevelSetFoliation(
                            level_set_edge_forward,
                            condNC_constraints,
                            paramNC_constraints,
                        )
                        self._add_constraints_to_transition(
                            level_set_edge_forward, sf.foliation
                        )

                    if i == nTransitions - 1 and crossedFoliation:
                        edgeName = nb_ls = nb + "_ls"
                        # containing state is goal state if an object in placement is grasped, start state otherwise
                        if not noPlace:
                            containingState = st.state_obj
                            level_set_edge_backward = (
                                self.graph.createLevelSetTransition(
                                    wStateObjects[i + 1],
                                    wStateObjects[i],
                                    edgeName,
                                    -1,
                                    containingState,
                                )
                            )
                            self.edge_objects[edgeName] = level_set_edge_backward

                            pNC_constraints = self._get_constraint_objects(
                                (sf.foliation - st.foliation).numConstraints
                            )
                            cNC_constraints = self._get_constraint_objects(
                                sf.manifold.numConstraints
                            )

                            self.graph.addLevelSetFoliation(
                                level_set_edge_backward,
                                cNC_constraints,
                                pNC_constraints,
                            )
                            self._add_constraints_to_transition(
                                level_set_edge_backward, st.foliation
                            )

                    if forward_ls_edge:
                        edge_to_use_forward = (
                            self.edge_objects[nf_ls] if i == 0 else forward_trans
                        )
                        self.graph.setWaypoint(
                            forward_ls_edge,
                            i,
                            edge_to_use_forward,
                            wStateObjects[i + 1],
                        )

                    if backward_ls_edge and not noPlace:
                        edge_to_use_backward = (
                            self.edge_objects[nb_ls]
                            if i == nTransitions - 1
                            else backward_trans
                        )
                        self.graph.setWaypoint(
                            backward_ls_edge,
                            nTransitions - 1 - i,
                            edge_to_use_backward,
                            wStateObjects[i],
                        )

                # Set waypoints for regular edges
                self.graph.setWaypoint(
                    forward_edge, i, forward_trans, wStateObjects[i + 1]
                )
                self.graph.setWaypoint(
                    backward_edge,
                    nTransitions - 1 - i,
                    backward_trans,
                    wStateObjects[i],
                )

                wTransitions.append((nf, nb))
                wTransitionObjects.append((forward_trans, backward_trans))

            M = 0 if gc.empty() else 1 + pregrasp

            for i in range(M):
                forward_trans, backward_trans = wTransitionObjects[i]

                self.graph.setContainingNode(forward_trans, sf.state_obj)
                self._add_constraints_to_transition(forward_trans, sf.foliation)

                self.graph.setContainingNode(backward_trans, sf.state_obj)
                self._add_constraints_to_transition(backward_trans, sf.foliation)

            for i in range(M, nTransitions):
                forward_trans, backward_trans = wTransitionObjects[i]

                self.graph.setContainingNode(forward_trans, st.state_obj)
                self._add_constraints_to_transition(forward_trans, st.foliation)

                self.graph.setContainingNode(backward_trans, st.state_obj)
                self._add_constraints_to_transition(backward_trans, st.foliation)

            if pregrasp:
                forward_trans, backward_trans = wTransitionObjects[1]
                self._add_constraints_to_transition(forward_trans, gcc)
                self._add_constraints_to_transition(backward_trans, gcc)

            if intersec and preplace and self.preplaceGuide:
                forward_trans, backward_trans = wTransitionObjects[pregrasp + intersec]
                self._add_constraints_to_transition(forward_trans, pcc)
                self._add_constraints_to_transition(backward_trans, pcc)

            for i in range(nTransitions - 1):
                forward_trans, backward_trans = wTransitionObjects[i + 1]
                self.graph.setShort(forward_trans, True)

                forward_trans_back, backward_trans_back = wTransitionObjects[i]
                self.graph.setShort(backward_trans_back, True)

        else:
            raise NotImplementedError("This case has not been implemented")

        self.transitions.add(names)
        # # \}

        # # \name Tuning the constraints
        #  \{

    def setPreplacementDistance(self, obj, distance):
        """
        Set preplacement distance
        \\param obj name of the object
        \\param distance distance of object to surface in preplacement
        configuration
        """
        # check that object is registered in factory
        if obj not in self.objects:
            raise RuntimeError(
                obj + " is not references as an object in the " + "factory"
            )
        self.preplaceDistance[obj] = distance

    def getPreplacementDistance(self, obj):
        """
        Get preplacement distance
        \\param obj name of the object
        """
        # check that object is registered in factory
        if obj not in self.objects:
            raise RuntimeError(
                obj + " is not references as an object in the " + "factory"
            )
        return self.preplaceDistance.get(obj, self.defaultPreplaceDist)

    # Set whether we should add placement complement
    # to the transition between intersec and preplace (if available)
    def setPreplaceGuide(self, preplaceGuide):
        self.preplaceGuide = preplaceGuide

    # # \}

    def _add_constraints_to_state(self, state_obj, constraints):
        """Convert Constraints object to constraint objects and add to state"""
        constraint_objects = []

        for constraint_name in constraints.numConstraints:
            if constraint_name in self.constraints.available_constraints:
                constraint_obj = self.constraints.available_constraints[constraint_name]
                constraint_objects.append(constraint_obj)

        # Handle grasps
        for grasp_name in constraints.grasps:
            if grasp_name in self.constraints.available_constraints:
                constraint_obj = self.constraints.available_constraints[grasp_name]
                constraint_objects.append(constraint_obj)

        # Handle pregrasps
        for pregrasp_name in constraints.pregrasps:
            if pregrasp_name in self.constraints.available_constraints:
                constraint_obj = self.constraints.available_constraints[pregrasp_name]
                constraint_objects.append(constraint_obj)

        if constraint_objects:
            self.graph.addNumericalConstraintsToState(state_obj, constraint_objects)

    def _add_constraints_to_transition(self, edge_obj, constraints):
        """Convert Constraints object to constraint objects and add to transition"""
        constraint_objects = []

        for constraint_name in constraints.numConstraints:
            if constraint_name in self.constraints.available_constraints:
                constraint_obj = self.constraints.available_constraints[constraint_name]
                constraint_objects.append(constraint_obj)

        if constraint_objects:
            self.graph.addNumericalConstraintsToTransition(edge_obj, constraint_objects)

    def _get_constraint_objects(self, constraint_names):
        """Convert list of constraint names to list of constraint objects"""
        constraint_objects = []
        for name in constraint_names:
            if name in self.constraints.available_constraints:
                constraint_objects.append(self.constraints.available_constraints[name])
        return constraint_objects


class Rule:
    def __init__(self, grippers=None, handles=None, link=False):
        self.grippers = grippers if grippers is not None else []
        self.handles = handles if handles is not None else []
        self.link = link
