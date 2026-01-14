#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#
# Test fixtures for hpp-python unit tests.
# Provides factory functions that create test objects.

import numpy as np
from math import pi
from pinocchio import SE3, Quaternion

from pyhpp.pinocchio import Device as CoreDevice, urdf as core_urdf
from pyhpp.core import Problem as CoreProblem
from pyhpp.manipulation import Device, urdf, Graph, Problem
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.constraints import (
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
    LockedJoint,
)


UR5_URDF = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
UR5_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"


def create_ur5_problem():
    """Create a UR5 robot and core Problem for non-manipulation tests."""
    robot = CoreDevice("ur5")
    core_urdf.loadModel(robot, 0, "ur5", "anchor", UR5_URDF, UR5_SRDF, SE3.Identity())
    problem = CoreProblem(robot)
    return problem, robot


# Robot URDF/SRDF paths
UR3_URDF = "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
UR3_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
SPHERE_URDF = "package://hpp_environments/urdf/construction_set/sphere.urdf"
SPHERE_SRDF = "package://hpp_environments/srdf/construction_set/sphere.srdf"
GROUND_URDF = "package://hpp_environments/urdf/construction_set/ground.urdf"
GROUND_SRDF = "package://hpp_environments/srdf/construction_set/ground.srdf"


def create_ur3_robot():
    """Load a basic UR3 robot without manipulation objects."""
    robot = Device("ur3-test")
    ur3_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "ur3", "anchor", UR3_URDF, UR3_SRDF, ur3_pose)
    robot.setJointBounds("ur3/shoulder_pan_joint", [-pi, 4])
    robot.setJointBounds("ur3/shoulder_lift_joint", [-pi, 0])
    robot.setJointBounds("ur3/elbow_joint", [-2.6, 2.6])
    return robot


def _create_placement_constraints(robot, objects):
    """Helper to create placement constraints for objects."""
    constraints = {}
    model = robot.model()

    for obj in objects:
        placement_name = f"place_{obj}"
        Id = SE3.Identity()
        q = Quaternion(1, 0, 0, 0)
        ball_placement = SE3(q, np.array([0, 0, 0.02]))
        joint = model.getJointId(f"{obj}/root_joint")

        # Placement constraint
        pc = Transformation(
            placement_name,
            robot,
            joint,
            Id,
            ball_placement,
            [False, False, True, True, True, False],
        )
        cts = ComparisonTypes()
        cts[:] = (
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
        )
        constraints[placement_name] = Implicit(pc, cts, [True, True, True])

        # Placement complement
        pc_comp = Transformation(
            placement_name + "/complement",
            robot,
            joint,
            Id,
            ball_placement,
            [True, True, False, False, False, True],
        )
        cts_comp = ComparisonTypes()
        cts_comp[:] = (
            ComparisonType.Equality,
            ComparisonType.Equality,
            ComparisonType.Equality,
        )
        constraints[placement_name + "/complement"] = Implicit(
            pc_comp, cts_comp, [True, True, True]
        )

        # Hold constraint (LockedJoint)
        cts_hold = ComparisonTypes()
        cts_hold[:] = (
            ComparisonType.Equality,
            ComparisonType.Equality,
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
            ComparisonType.Equality,
        )
        ll = LockedJoint(
            robot,
            f"{obj}/root_joint",
            np.array([0, 0, 0.02, 0, 0, 0, 1]),
            cts_hold,
        )
        constraints[placement_name + "/hold"] = ll

        # Pre-placement constraint
        preplace_name = f"preplace_{obj}"
        ball_preplace = SE3(q, np.array([0, 0, 0.1]))
        pc_pre = Transformation(
            preplace_name,
            robot,
            joint,
            Id,
            ball_preplace,
            [False, False, True, True, True, False],
        )
        cts_pre = ComparisonTypes()
        cts_pre[:] = (
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
            ComparisonType.EqualToZero,
        )
        constraints[preplace_name] = Implicit(pc_pre, cts_pre, [True, True, True])

    return constraints


def create_ur3_with_spheres():
    """
    Load UR3 with two spheres for manipulation testing.
    Returns (robot, objects, constraints) tuple.
    """
    robot = Device("ur3-spheres-test")

    # Load UR3
    ur3_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(robot, 0, "ur3", "anchor", UR3_URDF, UR3_SRDF, ur3_pose)

    # Load spheres
    n_spheres = 2
    objects = []
    for i in range(n_spheres):
        sphere_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
        urdf.loadModel(
            robot,
            0,
            f"sphere{i}",
            "freeflyer",
            SPHERE_URDF,
            SPHERE_SRDF,
            sphere_pose,
        )
        robot.setJointBounds(
            f"sphere{i}/root_joint",
            [
                -1.0,
                1.0,
                -1.0,
                1.0,
                -0.1,
                1.0,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001,
            ],
        )
        objects.append(f"sphere{i}")

    # Load ground
    ground_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    urdf.loadModel(
        robot, 0, "kitchen_area", "anchor", GROUND_URDF, GROUND_SRDF, ground_pose
    )

    # Set robot joint bounds
    robot.setJointBounds("ur3/shoulder_pan_joint", [-pi, 4])
    robot.setJointBounds("ur3/shoulder_lift_joint", [-pi, 0])
    robot.setJointBounds("ur3/elbow_joint", [-2.6, 2.6])

    # Create constraints
    constraints = _create_placement_constraints(robot, objects)

    return robot, objects, constraints


def create_constraint_graph_setup():
    """
    Create a complete constraint graph setup with factory.
    Returns (problem, graph, factory, robot, objects) tuple.
    """
    robot, objects, constraints = create_ur3_with_spheres()

    problem = Problem(robot)
    graph = Graph("test-graph", robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(0.0001)

    # Register constraints with graph
    for obj in objects:
        graph.registerConstraints(
            constraints[f"place_{obj}"],
            constraints[f"place_{obj}/complement"],
            constraints[f"place_{obj}/hold"],
        )

    # Setup handle masks (similar to graph_factory2.py pattern)
    handles = robot.handles()
    for obj in objects:
        handle_name = f"{obj}/handle"
        if handle_name in handles:
            h = handles[handle_name]
            h.mask = [True, True, True, False, True, True]

    # Create factory
    factory = ConstraintGraphFactory(graph, constraints)
    grippers = ["ur3/gripper"]
    handles_per_object = [[f"{obj}/handle"] for obj in objects]
    contacts_per_object = [[] for _ in objects]

    factory.setGrippers(grippers)
    factory.setObjects(objects, handles_per_object, contacts_per_object)
    factory.generate()

    graph.initialize()

    return problem, graph, factory, robot, objects


def create_security_margins_instance():
    """
    Create a SecurityMargins instance with basic configuration.
    Returns (security_margins, problem, graph, factory, robot, objects) tuple.
    """
    problem, graph, factory, robot, objects = create_constraint_graph_setup()
    robots_and_objects = ["ur3"] + objects

    sm = SecurityMargins(problem, factory, robots_and_objects, robot)

    return sm, problem, graph, factory, robot, objects
