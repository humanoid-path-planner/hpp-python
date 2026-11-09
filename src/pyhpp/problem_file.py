#
# Copyright (c) 2026 CNRS
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

"""Build a planning problem from a problem file (format hpp-problem/0.1).

A problem file is a TOML file (or any mapping) that describes what scene scripts
set in Python: robots and objects, joint bounds, handles, numerical
constraints, the constraint graph given to ConstraintGraphFactory, problem
components, initial and goal configurations, and the planner. Names are HPP
names (<robot>/<joint>), units metres and radians, poses
[x, y, z, qx, qy, qz, qw]. Without a graph section the robot and problem
are those of pyhpp.pinocchio and pyhpp.core.

Top-level keys:

  format: hpp-problem/0.1
  name, description
  robots: [{name, urdf, srdf (optional), root_joint, pose}], loaded in order
  joint_bounds: {joint: [lower, upper, ...]}
  handles: {handle: {mask}} overrides an SRDF handle; {link, pose, clearance,
      mask} adds one; approaching_direction in both cases
  partial_coms: {com: [joints]}
  constraints: {name: {type, ...}}, types:
      position, orientation, transformation: frame 2 (joint2 + frame2, or
          handle2 / gripper2) with respect to frame 1 in the world (frame1),
          mask, comparison (default EqualToZero)
      relative_position, relative_orientation, relative_transformation,
          relative_transformation_r3xso3: frame 2 with respect to frame 1
          (joint1 + frame1, or handle1 / gripper1)
      locked_joint: joint, value, comparison
      grasp: gripper, handle
      static_stability, sliding_stability, aligned_com_stability (sliding):
          com, left_ankle, right_ankle, config; create <name>/relative-com,
          <name>/pose-left-foot, ... as StaticStabilityConstraintsFactory
  graph: name, error_threshold, max_iterations, grippers,
      environment_contacts, objects [{name, handles, contacts}],
      rules [{grippers, handles, link}], possible_grasps {gripper: [handles]},
      placements {object: {constraint, complement, hold, preplacement}},
      add {graph: [constraints], transitions: {transition: [constraints]}},
      weights {transition: weight}, short [transitions],
      security_margins {robots, default, between [[a, b, margin]]}
          (SecurityMargins), transition_margins [{transitions (or all),
          pairs [[joint1, joint2]], margin}]
  problem: steering_method, path_validation {type, tolerance, graph
      (default true: GraphPathValidation), factory (graph: edge factory
      also wrapped)}, path_projector {type: Progressive, step},
      config_validations, config_projector {name, constraints, priorities},
      parameters {name: value}
  init, goal: {joints: {joint: value}} over the neutral configuration or
      {config: [...]}, then optionally project_on: <state>, project: true
      (problem constraints) or project_with {constraints, error_threshold,
      max_iterations}
  planner: type, max_iterations, parameters

Steering method and path validation keep the HPP defaults when not given.
Unknown keys raise ValueError.
"""

from dataclasses import dataclass

import numpy as np
import pinocchio
from pinocchio import SE3, Quaternion

FORMAT = "hpp-problem/0."

KEYS = {
    "problem file": {
        "format",
        "name",
        "description",
        "robots",
        "joint_bounds",
        "handles",
        "partial_coms",
        "constraints",
        "graph",
        "problem",
        "init",
        "goal",
        "planner",
    },
    "robot": {"name", "urdf", "srdf", "root_joint", "pose"},
    "handle": {"link", "pose", "clearance", "mask", "approaching_direction"},
    "constraint": {
        "type",
        "joint",
        "joint1",
        "joint2",
        "frame1",
        "frame2",
        "handle",
        "handle1",
        "handle2",
        "gripper",
        "gripper1",
        "gripper2",
        "mask",
        "comparison",
        "value",
        "com",
        "left_ankle",
        "right_ankle",
        "config",
        "sliding",
    },
    "graph": {
        "name",
        "error_threshold",
        "max_iterations",
        "grippers",
        "environment_contacts",
        "objects",
        "rules",
        "possible_grasps",
        "placements",
        "add",
        "weights",
        "short",
        "security_margins",
        "transition_margins",
    },
    "problem": {
        "steering_method",
        "path_validation",
        "path_projector",
        "config_validations",
        "config_projector",
        "parameters",
    },
    "object": {"name", "handles", "contacts"},
    "rule": {"grippers", "handles", "link"},
    "placement": {"constraint", "complement", "hold", "preplacement"},
    "add": {"graph", "transitions"},
    "security margins": {"robots", "default", "between"},
    "transition margins": {"transitions", "pairs", "margin"},
    "path validation": {"type", "tolerance", "graph", "factory"},
    "path projector": {"type", "step"},
    "config projector": {"name", "constraints", "priorities"},
    "configuration": {"joints", "config", "project_on", "project", "project_with"},
    "project with": {"constraints", "error_threshold", "max_iterations"},
    "planner": {"type", "max_iterations", "parameters"},
}

STABILITY = {
    "static_stability": "createStaticStabilityConstraint",
    "sliding_stability": "createSlidingStabilityConstraint",
    "aligned_com_stability": "createAlignedCOMStabilityConstraint",
}

# Absolute types constrain frame 2 (in joint 2) with respect to frame 1 in
# the world, relative ones with respect to frame 1 in joint 1.
ABSOLUTE = {
    "position": "Position",
    "orientation": "Orientation",
    "transformation": "Transformation",
}
RELATIVE = {
    "relative_position": "RelativePosition",
    "relative_orientation": "RelativeOrientation",
    "relative_transformation": "RelativeTransformation",
    "relative_transformation_r3xso3": "RelativeTransformationR3xSO3",
}


@dataclass
class ProblemFile:
    """Objects built from a problem file. init, goal and planner are None
    when the file does not give them."""

    name: str
    robot: object
    problem: object
    graph: object
    planner: object
    constraints: dict
    init: object
    goal: object


def check_keys(what, spec):
    unknown = set(spec) - KEYS[what]
    if unknown:
        raise ValueError(f"unknown keys in {what}: {sorted(unknown)}")


def joint_id(robot, name):
    model = robot.model()
    if not model.existJointName(name):
        raise ValueError(f"unknown joint {name}")
    return model.getJointId(name)


def pose(p):
    p = p or [0, 0, 0, 0, 0, 0, 1]
    return SE3(Quaternion(p[6], p[3], p[4], p[5]), np.array(p[:3]))


def comparison(names):
    from pyhpp.constraints import ComparisonType, ComparisonTypes

    cts = ComparisonTypes()
    cts[:] = tuple(getattr(ComparisonType, n) for n in names)
    return cts


def configuration(robot, c):
    """Full vector, or {joints: {name: value}} over the neutral configuration."""
    if isinstance(c, list):
        return np.array(c, dtype=float)
    model = robot.model()
    q = pinocchio.neutral(model)
    for joint, value in c["joints"].items():
        j = joint_id(robot, joint)
        q[model.idx_qs[j] : model.idx_qs[j] + model.nqs[j]] = value
    return q


def frame(robot, c, i):
    """Joint index and frame i: joint<i> and frame<i>, or the parent joint
    and local position of handle<i> or gripper<i>; joint 0 is the world."""
    if f"handle{i}" in c:
        h = robot.handles()[c[f"handle{i}"]]
    elif f"gripper{i}" in c:
        h = robot.grippers()[c[f"gripper{i}"]]
    else:
        joint = c.get(f"joint{i}")
        return (joint_id(robot, joint) if joint else 0), pose(c.get(f"frame{i}"))
    return h.getParentJointId(), h.localPosition


def constraint(name, c, robot, problem):
    """Numerical constraint, or a dict of them for the stability types."""
    from pyhpp import constraints

    check_keys("constraint", c)
    kind = c["type"]
    if kind in ABSOLUTE or kind in RELATIVE:
        mask = [bool(m) for m in c["mask"]]
        j1, f1 = frame(robot, c, 1)
        j2, f2 = frame(robot, c, 2)
        if kind in ABSOLUTE:
            f = getattr(constraints, ABSOLUTE[kind])(name, robot, j2, f2, f1, mask)
        else:
            f = getattr(constraints, RELATIVE[kind])(name, robot, j1, j2, f1, f2, mask)
        cts = c.get("comparison") or ["EqualToZero"] * sum(mask)
        return constraints.Implicit(f, comparison(cts), [True] * len(cts))
    if kind == "grasp":
        return robot.handles()[c["handle"]].createGrasp(
            robot.grippers()[c["gripper"]], name
        )
    if kind in STABILITY:
        from pyhpp.core.static_stability_constraint_factory import (
            StaticStabilityConstraintsFactory,
        )

        factory = StaticStabilityConstraintsFactory(problem, robot)
        args = [
            name + "/",
            c["com"],
            c["left_ankle"],
            c["right_ankle"],
            configuration(robot, c["config"]),
        ]
        if kind == "aligned_com_stability":
            args.append(c.get("sliding", False))
        return getattr(factory, STABILITY[kind])(*args)
    if kind == "locked_joint":
        value = np.array(c["value"], dtype=float)
        if "comparison" in c:
            return constraints.LockedJoint(
                robot, c["joint"], value, comparison(c["comparison"])
            )
        return constraints.LockedJoint(robot, c["joint"], value)
    raise ValueError(f"unknown constraint type {kind}")


def graph_from(g, robot, problem, constraints):
    """Constraint graph generated by ConstraintGraphFactory, not initialized."""
    from pyhpp.manipulation import Graph
    from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule

    check_keys("graph", g)
    graph = Graph(g.get("name", "graph"), robot, problem)
    graph.errorThreshold(g.get("error_threshold", 1e-4))
    graph.maxIterations(g.get("max_iterations", 40))
    factory_constraints = dict(constraints)
    for obj, pl in g.get("placements", {}).items():
        check_keys("placement", pl)
        # The factory finds placements by name: place_<obj>, place_<obj>/complement, preplace_<obj>.
        for key, conventional in (
            ("constraint", f"place_{obj}"),
            ("complement", f"place_{obj}/complement"),
            ("preplacement", f"preplace_{obj}"),
        ):
            if key in pl:
                factory_constraints[conventional] = constraints[pl[key]]
        if "hold" in pl:
            graph.registerConstraints(
                constraints[pl["constraint"]],
                constraints[pl["complement"]],
                constraints[pl["hold"]],
            )
    factory = ConstraintGraphFactory(graph, factory_constraints)
    factory.setGrippers(g["grippers"])
    if g.get("environment_contacts"):
        factory.environmentContacts(g["environment_contacts"])
    objects = g.get("objects", [])
    for o in objects:
        check_keys("object", o)
    factory.setObjects(
        [o["name"] for o in objects],
        [o["handles"] for o in objects],
        [o.get("contacts", []) for o in objects],
    )
    if "rules" in g:
        for r in g["rules"]:
            check_keys("rule", r)
        factory.setRules(
            [Rule(r["grippers"], r["handles"], r["link"]) for r in g["rules"]]
        )
    if "possible_grasps" in g:
        factory.setPossibleGrasps(g["possible_grasps"])
    factory.generate()
    add = g.get("add", {})
    check_keys("add", add)
    if add.get("graph"):
        graph.addNumericalConstraintsToGraph([constraints[n] for n in add["graph"]])
    for transition, names in add.get("transitions", {}).items():
        graph.addNumericalConstraintsToTransition(
            graph.getTransition(transition), [constraints[n] for n in names]
        )
    for transition, weight in g.get("weights", {}).items():
        graph.setWeight(graph.getTransition(transition), weight)
    margins = g.get("security_margins")
    if margins:
        check_keys("security margins", margins)
        from pyhpp.manipulation.security_margins import SecurityMargins

        sm = SecurityMargins(problem, factory, margins["robots"], robot)
        sm.defaultMargin = margins.get("default", 0.0)
        for a, b, m in margins.get("between", []):
            sm.setSecurityMarginBetween(a, b, m)
        sm.apply()
    for m in g.get("transition_margins", []):
        check_keys("transition margins", m)
        names = (
            graph.getTransitionNames()
            if m["transitions"] == "all"
            else m["transitions"]
        )
        for t in names:
            for j1, j2 in m["pairs"]:
                graph.setSecurityMarginForTransition(
                    graph.getTransition(t), j1, j2, m["margin"]
                )
    for t in g.get("short", []):
        graph.setShort(graph.getTransition(t), True)
    return graph


def load(spec):
    """Build robot, problem, constraint graph and planner from a problem
    file read as a mapping."""
    from pyhpp import core

    check_keys("problem file", spec)
    if not str(spec.get("format", "")).startswith(FORMAT):
        raise ValueError(f"expected format {FORMAT}x, got {spec.get('format')}")
    if spec.get("graph"):
        import pyhpp.manipulation as module
        from pyhpp.manipulation import Device, Problem, urdf
    else:
        module = core
        from pyhpp.core import Problem
        from pyhpp.pinocchio import Device, urdf

    robot = Device(spec["name"])
    for r in spec["robots"]:
        check_keys("robot", r)
        urdf.loadModel(
            robot,
            0,
            r["name"],
            r["root_joint"],
            r["urdf"],
            r.get("srdf", ""),
            pose(r.get("pose")),
        )
    for joint, bounds in spec.get("joint_bounds", {}).items():
        robot.setJointBounds(joint, bounds)
    for handle, h in spec.get("handles", {}).items():
        check_keys("handle", h)
        if "link" in h:
            robot.addHandle(
                h["link"],
                handle,
                pose(h["pose"]),
                h["clearance"],
                [bool(m) for m in h["mask"]],
            )
        elif "mask" in h:
            robot.handles()[handle].mask = [bool(m) for m in h["mask"]]
        if "approaching_direction" in h:
            robot.handles()[handle].approachingDirection = np.array(
                h["approaching_direction"], dtype=float
            )

    problem = Problem(robot)
    p = spec.get("problem", {})
    check_keys("problem", p)
    if "config_validations" in p:
        problem.clearConfigValidations()
        for v in p["config_validations"]:
            problem.addConfigValidation(v)
    for com, joints in spec.get("partial_coms", {}).items():
        problem.addPartialCom(com, joints)
    constraints = {}
    for n, c in spec.get("constraints", {}).items():
        made = constraint(n, c, robot, problem)
        constraints.update(made if isinstance(made, dict) else {n: made})
    cp = p.get("config_projector")
    if cp:
        check_keys("config projector", cp)
        names = cp["constraints"]
        problem.addNumericalConstraintsToConfigProjector(
            cp["name"],
            [constraints[n] for n in names],
            cp.get("priorities", [0] * len(names)),
        )

    graph = (
        graph_from(spec["graph"], robot, problem, constraints)
        if spec.get("graph")
        else None
    )

    if "steering_method" in p:
        problem.steeringMethod(getattr(core, p["steering_method"])(problem))
    pv = p.get("path_validation")
    if pv:
        check_keys("path validation", pv)
        validation = lambda: getattr(core, pv["type"])(robot, pv.get("tolerance", 0))
        if graph:
            graph_validation = lambda: module.GraphPathValidation(validation())
            problem.pathValidation(
                graph_validation() if pv.get("graph", True) else validation()
            )
            problem.pathValidationFactory(
                graph_validation() if pv.get("factory") == "graph" else validation()
            )
        else:
            problem.pathValidation(validation())
    pp = p.get("path_projector")
    if pp:
        check_keys("path projector", pp)
        if pp["type"] != "Progressive":
            raise ValueError(f"unknown path projector {pp['type']}")
        problem.pathProjector(
            module.ProgressiveProjector(
                problem.distance(), problem.steeringMethod(), pp["step"]
            )
        )
    if graph:
        graph.initialize()

    def config(c):
        check_keys("configuration", c)
        q = configuration(robot, c.get("config", c))
        if c.get("project_on"):
            ok, q, _ = graph.applyStateConstraints(graph.getState(c["project_on"]), q)
            if not ok:
                raise RuntimeError(
                    f"Failed to project configuration on {c['project_on']}"
                )
        if c.get("project"):
            ok, q, _ = problem.applyConstraints(q)
            if not ok:
                raise RuntimeError(
                    "Failed to project configuration on the problem constraints"
                )
        pw = c.get("project_with")
        if pw:
            check_keys("project with", pw)
            solver = core.ConfigProjector(
                robot,
                "solver",
                pw.get("error_threshold", 1e-4),
                pw.get("max_iterations", 40),
            )
            for n in pw["constraints"]:
                solver.add(constraints[n], 0)
            q, ok = solver.solver().solve(q)
            if not ok:
                raise RuntimeError(
                    f"Failed to project configuration with {pw['constraints']}"
                )
        return q

    init = config(spec["init"]) if "init" in spec else None
    goal = config(spec["goal"]) if "goal" in spec else None
    if init is not None:
        problem.initConfig(init)
    if goal is not None:
        problem.addGoalConfig(goal)
    if graph:
        problem.constraintGraph(graph)

    pl = spec.get("planner", {})
    check_keys("planner", pl)
    planner = None
    if "type" in pl:
        planner = (getattr(module, pl["type"], None) or getattr(core, pl["type"]))(
            problem
        )
        if "max_iterations" in pl:
            planner.maxIterations(pl["max_iterations"])
    for k, v in {**p.get("parameters", {}), **pl.get("parameters", {})}.items():
        problem.setParameter(k, v)
    return ProblemFile(
        spec["name"], robot, problem, graph, planner, constraints, init, goal
    )


def load_file(path):
    """load() of a TOML file (Python 3.11 or later, for tomllib)."""
    import tomllib

    with open(path, "rb") as f:
        return load(tomllib.load(f))
