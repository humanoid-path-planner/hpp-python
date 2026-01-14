#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#

import unittest
import numpy as np
from unit.conftest import create_ur5_problem
from pyhpp.core import Roadmap, WeighedDistance


class TestRoadmap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()
        cls.distance = WeighedDistance(cls.robot)

    def test_init_creates_one_component(self):
        roadmap = Roadmap(self.distance, self.robot)
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        roadmap.initNode(q_init)

        self.assertEqual(roadmap.numberConnectedComponents(), 1)

    def test_goal_creates_separate_component(self):
        roadmap = Roadmap(self.distance, self.robot)
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_goal = np.array([1.57, -1.57, -1.8, 0.0, 0.8, 0.0])

        roadmap.initNode(q_init)
        roadmap.addGoalNode(q_goal)

        self.assertEqual(roadmap.numberConnectedComponents(), 2)

    def test_nearest_node_returns_closest(self):
        roadmap = Roadmap(self.distance, self.robot)
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        roadmap.initNode(q_init)

        q_query = np.array([0.1, -1.57, -1.8, 0.0, 0.8, 0.0])
        cc = roadmap.getConnectedComponent(0)
        q_near, dist = roadmap.nearestNode(q_query, cc)

        np.testing.assert_array_almost_equal(q_near, q_init)
        self.assertGreater(dist, 0)

    def test_add_node_increases_component_count(self):
        roadmap = Roadmap(self.distance, self.robot)
        q_init = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q_other = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        roadmap.initNode(q_init)
        self.assertEqual(roadmap.numberConnectedComponents(), 1)

        roadmap.addNode(q_other)
        self.assertEqual(roadmap.numberConnectedComponents(), 2)

    def test_add_edge_creates_connection(self):
        """Adding an edge between nodes should create a connection."""
        roadmap = Roadmap(self.distance, self.robot)
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        roadmap.initNode(q1)
        roadmap.addNode(q2)

        nodes = roadmap.nodes()
        self.assertEqual(len(nodes), 2)
        node1, node2 = nodes[0], nodes[1]

        steer = self.problem.steeringMethod()
        path = steer(q1, q2)
        roadmap.addEdge(node1, node2, path)

        # Edge was added successfully (no exception raised)
        self.assertEqual(len(roadmap.nodes()), 2)


class TestRoadmapNegativeCases(unittest.TestCase):
    """Negative test cases for Roadmap operations."""

    @classmethod
    def setUpClass(cls):
        cls.problem, cls.robot = create_ur5_problem()
        cls.distance = WeighedDistance(cls.robot)

    def test_empty_roadmap_has_zero_components(self):
        """Empty roadmap should have zero connected components."""
        roadmap = Roadmap(self.distance, self.robot)

        self.assertEqual(roadmap.numberConnectedComponents(), 0)

    def test_empty_roadmap_nodes_list_empty(self):
        """Empty roadmap should have no nodes."""
        roadmap = Roadmap(self.distance, self.robot)

        self.assertEqual(len(roadmap.nodes()), 0)

    def test_multiple_nodes_tracked(self):
        """Adding multiple nodes should increase node count."""
        roadmap = Roadmap(self.distance, self.robot)
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])
        q3 = np.array([1.0, -1.57, -1.8, 0.0, 0.8, 0.0])

        roadmap.initNode(q1)
        roadmap.addNode(q2)
        roadmap.addNode(q3)

        self.assertEqual(len(roadmap.nodes()), 3)

    def test_clear_resets_roadmap(self):
        """Clear should reset the roadmap to empty state."""
        roadmap = Roadmap(self.distance, self.robot)
        q1 = np.array([0.0, -1.57, -1.8, 0.0, 0.8, 0.0])
        q2 = np.array([0.5, -1.57, -1.8, 0.0, 0.8, 0.0])

        roadmap.initNode(q1)
        roadmap.addNode(q2)
        self.assertEqual(roadmap.numberConnectedComponents(), 2)

        roadmap.clear()
        self.assertEqual(roadmap.numberConnectedComponents(), 0)
        self.assertEqual(len(roadmap.nodes()), 0)


if __name__ == "__main__":
    unittest.main()
