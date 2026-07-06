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

// DocNamespace(hpp::core)

namespace {
const char* DOC_R_ADDNODE =
    "Add a node with the given configuration to the roadmap.";
const char* DOC_R_NEAREST1 =
    "Find nearest node to configuration (optionally searching in reverse "
    "direction). Returns (configuration, minDistance).";
const char* DOC_R_NEAREST3 =
    "Find nearest node within a connected component. Returns (configuration, "
    "minDistance).";
const char* DOC_R_NEARESTNODES1 =
    "Find the k nearest nodes to configuration. Returns a list of nodes.";
const char* DOC_R_NEARESTNODES2 =
    "Find the k nearest nodes within a connected component.";
const char* DOC_R_ADDNODEEDGES =
    "Add node at 'to' and edges from the node nearest to 'from'.";
const char* DOC_R_ADDNODEEDGE =
    "Add node at 'to' and a single directed edge from the node nearest to "
    "'from'.";
const char* DOC_R_ADDEDGE1 =
    "Add nodes at from and to, then add a directed edge between them.";
const char* DOC_R_ADDEDGE2 =
    "Add nodes at from and to; if bothEdges is true also adds the reverse "
    "edge.";
const char* DOC_R_NODES =
    "Return list of all node configurations in the roadmap.";
const char* DOC_R_NODESCC =
    "Return list of configurations in the connected component identified by "
    "connectedComponentId.";
const char* DOC_R_INITNODE1 =
    "Set the initial node to the given configuration.";
const char* DOC_R_INITNODE2 = "Return the current initial node object.";
const char* DOC_R_CCS = "Return list of all connected components.";
const char* DOC_R_NUMCCS = "Return the number of connected components.";
const char* DOC_R_GETCC = "Return connected component by index.";
const char* DOC_R_CCOFNODE =
    "Return the connected component of the node nearest to the given "
    "configuration.";
}  // namespace

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

  static ConnectedComponentPtr_t connectedComponentOfNode(
      Roadmap& roadmap, const ConfigurationIn_t q) {
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
  // DocClass(Roadmap)
  class_<Roadmap, RoadmapPtr_t, boost::noncopyable>("Roadmap", no_init)
      .def("__init__", make_constructor(&Roadmap::create))
      .def("__str__", &to_str<Roadmap>)
      .def("clear", &Roadmap::clear, DocClassMethod(clear))
      .def("addNode", &RWrapper::addNode,
           return_value_policy<reference_existing_object>(), DOC_R_ADDNODE)
      .def("nearestNode", &RWrapper::nearestNode1, DOC_R_NEAREST1)
      .def("nearestNode", &RWrapper::nearestNode2, DOC_R_NEAREST1)
      .def("nearestNode", &RWrapper::nearestNode3, DOC_R_NEAREST3)
      .def("nearestNode", &RWrapper::nearestNode4, DOC_R_NEAREST3)
      .def("nearestNodes", &RWrapper::nearestNodes1, DOC_R_NEARESTNODES1)
      .def("nearestNodes", &RWrapper::nearestNodes2, DOC_R_NEARESTNODES2)
      .def("nodesWithinBall", &Roadmap::nodesWithinBall,
           DocClassMethod(nodesWithinBall))
      .def("addNodeAndEdges", &RWrapper::addNodeAndEdges,
           return_value_policy<reference_existing_object>(), DOC_R_ADDNODEEDGES)
      .def("addNodeAndEdge", &RWrapper::addNodeAndEdge,
           return_value_policy<reference_existing_object>(), DOC_R_ADDNODEEDGE)
      .def("addEdge", &RWrapper::addEdge, DOC_R_ADDEDGE1)
      .def("addEdge", &RWrapper::addEdge2, DOC_R_ADDEDGE2)
      .def("addEdges", &Roadmap::addEdges, DocClassMethod(addEdges))
      .def("merge",
           static_cast<void (Roadmap::*)(const RoadmapPtr_t&)>(&Roadmap::merge),
           DocClassMethod(merge))
      .def("insertPathVector", &Roadmap::insertPathVector,
           DocClassMethod(insertPathVector))
      .def("addGoalNode", &Roadmap::addGoalNode,
           return_value_policy<reference_existing_object>(),
           DocClassMethod(addGoalNode))
      .def("resetGoalNodes", &Roadmap::resetGoalNodes,
           DocClassMethod(resetGoalNodes))
      .def("pathExists", &Roadmap::pathExists, DocClassMethod(pathExists))
      .def("nodes", &RWrapper::nodes, DOC_R_NODES)
      .def("nodesConnectedComponent", &RWrapper::nodesConnectedComponent,
           DOC_R_NODESCC)
      .def("initNode", &RWrapper::initNode1, DOC_R_INITNODE1)
      .def("initNode", &RWrapper::initNode2,
           return_value_policy<reference_existing_object>(), DOC_R_INITNODE2)
      .def("goalNodes", &Roadmap::goalNodes, return_internal_reference<>(),
           DocClassMethod(goalNodes))
      .def("connectedComponents", &RWrapper::connectedComponents, DOC_R_CCS)
      .def("distance", &Roadmap::distance, return_internal_reference<>(),
           DocClassMethod(distance))
      .def("numberConnectedComponents", &RWrapper::numberConnectedComponents,
           DOC_R_NUMCCS)
      .def("getConnectedComponent", &RWrapper::getConnectedComponent,
           DOC_R_GETCC)
      .def("connectedComponentOfNode", &RWrapper::connectedComponentOfNode,
           DOC_R_CCOFNODE);
}
}  // namespace core
}  // namespace pyhpp
