//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <boost/python.hpp>
#include <hpp/core/path.hh>
#include <hpp/core/roadmap.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {

using namespace hpp::core;

// struct CCWrapper {
//   static Nodes_t nodes(const ConnectedComponent& cc) {
//     return cc.nodes();
//   }
// };
struct RWrapper {
  static void addNodeAndEdges(Roadmap& roadmap, const ConfigurationIn_t from,
                              ConfigurationIn_t to, const PathPtr_t path) {
    value_type d;
    NodePtr_t nodeFrom = roadmap.nearestNode(from, d);
    if (d > 0) {
      std::ostringstream os;
      os << "Roadmap::addNodeAndEdge: initial configuration (" << from
         << ") not in the roadmap";
      throw std::logic_error(os.str().c_str());
    }
    roadmap.addNodeAndEdges(nodeFrom, to, path);
  }

  static ConnectedComponentPtr_t connectedComponentOfNode(Roadmap& roadmap,
      const ConfigurationIn_t q) {
    value_type d;
    NodePtr_t node = roadmap.nearestNode(q, d);
    if (d > 0) {
      std::ostringstream os;
      os << "Roadmap::connectedComponentOfNode: input configuration (" << q
         << ") not in the roadmap";
      throw std::logic_error(os.str().c_str());
    }
    return node->connectedComponent();
  }

  static void addNodeAndEdge(Roadmap& roadmap, const ConfigurationIn_t from,
                             ConfigurationIn_t to, const PathPtr_t path) {
    value_type d;
    NodePtr_t nodeFrom = roadmap.nearestNode(from, d);
    if (d > 0) {
      std::ostringstream os;
      os << "Roadmap::addNodeAndEdge: initial configuration (" << from
         << ") not in the roadmap";
      throw std::logic_error(os.str().c_str());
    }
    NodePtr_t nodeTo = roadmap.addNode(to);
    roadmap.addEdge(nodeFrom, nodeTo, path);
  }

  static void addNode(Roadmap& roadmap, const ConfigurationIn_t config) {
    roadmap.addNode(config);
  }

  static void addEdge(Roadmap& roadmap, const ConfigurationIn_t from,
                      ConfigurationIn_t to, const PathPtr_t& path) {
    NodePtr_t nodeFrom = roadmap.addNode(from);
    NodePtr_t nodeTo = roadmap.addNode(to);
    roadmap.addEdge(nodeFrom, nodeTo, path);
    return;
  }

  static void addEdge2(Roadmap& roadmap, const ConfigurationIn_t from,
                       ConfigurationIn_t to, const PathPtr_t& path,
                       bool bothEdges) {
    NodePtr_t nodeFrom = roadmap.addNode(from);
    NodePtr_t nodeTo = roadmap.addNode(to);
    if (bothEdges) {
      roadmap.addEdge(nodeFrom, nodeTo, path);
      roadmap.addEdge(nodeTo, nodeFrom, path->reverse());
      return;
    }
    roadmap.addEdge(nodeFrom, nodeTo, path);
    return;
  }

  static boost::python::tuple nearestNode1(Roadmap& roadmap,
                                           ConfigurationIn_t configuration,
                                           bool reverse) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, minDistance, reverse);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode2(Roadmap& roadmap,
                                           ConfigurationIn_t configuration) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, minDistance);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode3(
      Roadmap& roadmap, ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent, bool reverse) {
    double minDistance;
    NodePtr_t node = roadmap.nearestNode(configuration, connectedComponent,
                                         minDistance, reverse);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }
  static boost::python::tuple nearestNode4(
      Roadmap& roadmap, ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent) {
    double minDistance;
    NodePtr_t node =
        roadmap.nearestNode(configuration, connectedComponent, minDistance);
    return boost::python::make_tuple(node->configuration(), minDistance);
  }

  static Nodes_t nearestNodes1(Roadmap& roadmap,
                               ConfigurationIn_t configuration, size_type k) {
    return roadmap.nearestNodes(configuration, k);
  }
  static Nodes_t nearestNodes2(
      Roadmap& roadmap, ConfigurationIn_t configuration,
      const ConnectedComponentPtr_t& connectedComponent, size_type k) {
    return roadmap.nearestNodes(configuration, connectedComponent, k);
  }

  static void initNode1(Roadmap& roadmap, ConfigurationIn_t configuration) {
    roadmap.initNode(configuration);
    return;
  }
  static NodePtr_t initNode2(Roadmap& roadmap) { return roadmap.initNode(); }

  static int numberConnectedComponents(Roadmap& roadmap) {
    return (int)roadmap.connectedComponents().size();
  }

  static ConnectedComponentPtr_t getConnectedComponent(
      Roadmap& roadmap, int connectedComponentId) {
    ConnectedComponents_t::const_iterator itcc =
        roadmap.connectedComponents().begin();
    std::advance(itcc, connectedComponentId);
    return *itcc;
  }

  static boost::python::list connectedComponents(Roadmap& roadmap) {
    std::vector<ConnectedComponentPtr_t> res(
        roadmap.connectedComponents().begin(),
        roadmap.connectedComponents().end());
    return to_python_list(res);
  }
  static boost::python::list nodes(Roadmap& roadmap) {
    Configurations_t res;
    for (const auto& n : roadmap.nodes()) {
      res.push_back(n->configuration());
    }
    return to_python_list(res);
  }

  static boost::python::list nodesConnectedComponent(Roadmap& roadmap,
                                                     int connectedComponentId) {
    try {
      const ConnectedComponents_t& connectedComponents =
          roadmap.connectedComponents();

      if ((std::size_t)connectedComponentId >= connectedComponents.size()) {
        std::ostringstream oss;
        oss << "connectedComponentId=" << connectedComponentId
            << " out of range [0," << connectedComponents.size() - 1 << "].";
        throw std::runtime_error(oss.str());
      }

      ConnectedComponents_t::const_iterator itcc = connectedComponents.begin();
      std::advance(itcc, connectedComponentId);

      const NodeVector_t& nodes = (*itcc)->nodes();

      Configurations_t res;
      res.reserve(nodes.size());

      for (const auto& node : nodes) {
        res.push_back(node->configuration());
      }

      return to_python_list(res);

    } catch (const std::exception& exc) {
      throw std::runtime_error(exc.what());
    }
  }
};

void exposeRoadmap() {
  class_<Roadmap, RoadmapPtr_t, boost::noncopyable>("Roadmap", no_init)
      .def("__init__", make_constructor(&Roadmap::create))
      .def("__str__", &to_str<Roadmap>)
      .PYHPP_DEFINE_METHOD(Roadmap, clear)
      .PYHPP_DEFINE_METHOD1(RWrapper, addNode,
                            return_value_policy<reference_existing_object>())
      .def("nearestNode", &RWrapper::nearestNode1)
      .def("nearestNode", &RWrapper::nearestNode2)
      .def("nearestNode", &RWrapper::nearestNode3)
      .def("nearestNode", &RWrapper::nearestNode4)
      .def("nearestNodes", &RWrapper::nearestNodes1)
      .def("nearestNodes", &RWrapper::nearestNodes2)
      .PYHPP_DEFINE_METHOD(Roadmap, nodesWithinBall)
      .PYHPP_DEFINE_METHOD1(RWrapper, addNodeAndEdges,
                            return_value_policy<reference_existing_object>())
      .PYHPP_DEFINE_METHOD1(RWrapper, addNodeAndEdge,
                            return_value_policy<reference_existing_object>())
      .def("addEdge", &RWrapper::addEdge)
      .def("addEdge", &RWrapper::addEdge2)
      .PYHPP_DEFINE_METHOD(Roadmap, addEdges)
      .def("merge",
           static_cast<void (Roadmap::*)(const RoadmapPtr_t&)>(&Roadmap::merge))
      .PYHPP_DEFINE_METHOD(Roadmap, insertPathVector)
      .PYHPP_DEFINE_METHOD1(Roadmap, addGoalNode,
                            return_value_policy<reference_existing_object>())
      .PYHPP_DEFINE_METHOD(Roadmap, resetGoalNodes)
      .PYHPP_DEFINE_METHOD(Roadmap, pathExists)
      .def("nodes", &RWrapper::nodes)
      .def("nodesConnectedComponent", &RWrapper::nodesConnectedComponent)
      .def("initNode", &RWrapper::initNode1)
      .def("initNode", &RWrapper::initNode2,
           return_value_policy<reference_existing_object>())
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, goalNodes)
      .def("connectedComponents", &RWrapper::connectedComponents)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Roadmap, distance)
      .def("numberConnectedComponents", &RWrapper::numberConnectedComponents)
      .def("getConnectedComponent", &RWrapper::getConnectedComponent)
      .def("connectedComponentOfNode", &RWrapper::connectedComponentOfNode)
      // .def("cost", &RWrapper::cost1)
      // .def("cost", &RWrapper::cost2,
      // return_value_policy<reference_existing_object>())

      ;
}
}  // namespace core
}  // namespace pyhpp
