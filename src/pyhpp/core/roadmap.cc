//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <hpp/core/roadmap.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {

using namespace hpp::core;

struct RWrapper {
  static void addNodeAndEdges(Roadmap& roadmap, const ConfigurationIn_t from, ConfigurationIn_t to, const PathPtr_t path) {
    NodePtr_t nodeFrom = roadmap.addNode(from);
    roadmap.addNodeAndEdges(nodeFrom, to, path);
  }

  static void addNodeAndEdge(Roadmap& roadmap, const ConfigurationIn_t from, ConfigurationIn_t to, const PathPtr_t path) {
    NodePtr_t nodeFrom = roadmap.addNode(from);
    roadmap.addNodeAndEdge(nodeFrom, to, path);
  }

  static void addNode(Roadmap& roadmap, const ConfigurationIn_t config) {
    roadmap.addNode(config);
  }

  static void addEdge(Roadmap& roadmap, const ConfigurationIn_t from, ConfigurationIn_t to, const PathPtr_t &path) {
    NodePtr_t nodeFrom = roadmap.addNode(from);
    NodePtr_t nodeTo = roadmap.addNode(to);
    roadmap.addEdge(nodeFrom, nodeTo, path);
    return;
  }

  static boost::python::tuple nearestNode1(Roadmap& roadmap, ConfigurationIn_t configuration, bool reverse) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, minDistance, reverse);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode2(Roadmap& roadmap, ConfigurationIn_t configuration) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, minDistance);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode3(Roadmap& roadmap, ConfigurationIn_t configuration, const ConnectedComponentPtr_t &connectedComponent, bool reverse) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, connectedComponent, minDistance, reverse);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode4(Roadmap& roadmap, ConfigurationIn_t configuration, const ConnectedComponentPtr_t &connectedComponent) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, connectedComponent, minDistance);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }

  static Nodes_t nearestNodes1(Roadmap& roadmap, ConfigurationIn_t configuration, size_type k) {
    return roadmap.nearestNodes(configuration, k);
  }
  static Nodes_t nearestNodes2(Roadmap& roadmap, ConfigurationIn_t configuration, const ConnectedComponentPtr_t &connectedComponent, size_type k) {
    return roadmap.nearestNodes(configuration, connectedComponent, k);
  }

  static void initNode1(Roadmap& roadmap, ConfigurationIn_t configuration) {
    roadmap.initNode(configuration);
    return;
  }
  static NodePtr_t initNode2(Roadmap& roadmap) {
    return roadmap.initNode();
  }

  static int numberConnectedComponents(Roadmap& roadmap) {
    return roadmap.connectedComponents().size();
  }

  static ConnectedComponentPtr_t getConnectedComponent(Roadmap& roadmap, int connectedComponentId) {
     ConnectedComponents_t::const_iterator itcc =
          roadmap.connectedComponents().begin();
      std::advance(itcc, connectedComponentId);
    return *itcc;
  }

  // static void cost1(Roadmap& roadmap, const path::CostPtr_t &cost) {
  //   roadmap.cost(cost);
  //   return;
  // }
  // static path::CostPtr_t cost2(Roadmap& roadmap) {
  //   return roadmap.cost();
  // }
};

void exposeRoadmap() {

  class_<Roadmap, RoadmapPtr_t, boost::noncopyable>("Roadmap", no_init)
    .def("create", &Roadmap::create).staticmethod("create")
    .def("__str__", &to_str<Roadmap>)
    .PYHPP_DEFINE_METHOD(Roadmap, clear)
    .PYHPP_DEFINE_METHOD1(RWrapper, addNode, return_value_policy<reference_existing_object>())
    .def("nearestNode", &RWrapper::nearestNode1)
    .def("nearestNode", &RWrapper::nearestNode2)
    .def("nearestNode", &RWrapper::nearestNode3)
    .def("nearestNode", &RWrapper::nearestNode4)
    .def("nearestNodes", &RWrapper::nearestNodes1)
    .def("nearestNodes", &RWrapper::nearestNodes2)
    .PYHPP_DEFINE_METHOD(Roadmap, nodesWithinBall)
    .PYHPP_DEFINE_METHOD1(RWrapper, addNodeAndEdges, return_value_policy<reference_existing_object>())
    .PYHPP_DEFINE_METHOD1(RWrapper, addNodeAndEdge, return_value_policy<reference_existing_object>())
    .PYHPP_DEFINE_METHOD(RWrapper, addEdge)
    .PYHPP_DEFINE_METHOD(Roadmap, addEdges)
    .def("merge", static_cast<void (Roadmap::*)(const RoadmapPtr_t&)>(&Roadmap::merge))
    .PYHPP_DEFINE_METHOD(Roadmap, insertPathVector)
    .PYHPP_DEFINE_METHOD1(Roadmap, addGoalNode, return_value_policy<reference_existing_object>())
    .PYHPP_DEFINE_METHOD(Roadmap, resetGoalNodes)
    .PYHPP_DEFINE_METHOD(Roadmap, pathExists)
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, nodes)
    .def("initNode", &RWrapper::initNode1)
    .def("initNode", &RWrapper::initNode2, return_value_policy<reference_existing_object>())
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, goalNodes)
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, connectedComponents)
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, distance)
    .def("numberConnectedComponents", &RWrapper::numberConnectedComponents)
    .def("getConnectedComponent", &RWrapper::getConnectedComponent)
    // .def("cost", &RWrapper::cost1)
    // .def("cost", &RWrapper::cost2, return_value_policy<reference_existing_object>())



;
}
}  // namespace core
}  // namespace pyhpp