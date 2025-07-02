//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-python
// hpp-python is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/manipulation/graph/graph.hh>

namespace pyhpp {
namespace manipulation {

typedef hpp::manipulation::size_type size_type;
typedef hpp::manipulation::value_type value_type;
typedef hpp::manipulation::graph::GraphComponent GraphComponent;
typedef hpp::manipulation::graph::GraphComponentPtr_t GraphComponentPtr_t;
typedef hpp::manipulation::ProblemPtr_t ProblemPtr_t;
typedef hpp::manipulation::graph::Edge Edge;
typedef hpp::manipulation::graph::EdgePtr_t EdgePtr_t;
typedef hpp::manipulation::graph::State State;
typedef hpp::manipulation::graph::StatePtr_t StatePtr_t;

struct Graph {
  typedef hpp::constraints::ImplicitPtr_t ImplicitPtr_t;
  typedef hpp::constraints::NumericalConstraints_t NumericalConstraints_t;
  // Pointer to the underlying object
  hpp::manipulation::graph::GraphPtr_t obj;
  // Map from name to id
  std::map<std::string, std::size_t> id;
  // Get shared pointer to object from name among (Graph, Node, Edge)
  template <typename T>
  std::shared_ptr<T> getComp(const std::string& name);
  // Constructor with a pointer to underlying graph
  Graph(const hpp::manipulation::graph::GraphPtr_t& object);
  // Constructor exposed in python
  Graph(const std::string& name, const Device& d, const ProblemPtr_t& problem);
  // Get name of the graph
  std::string name() const;
  // Create a state in the graph
  std::size_t createState(const std::string& nodeName, bool waypoint,
                          int priority);
  // Create a transition between two states in the graph
  std::size_t createTransition(const std::string& nodeFrom,
                               const std::string& nodeTo,
                               const std::string& transitionName, int w,
                               const std::string& isInState);
  // Add numerical constraints to a graph, a state or a transition
  void addNumericalConstraint(const std::string& name,
                              const ImplicitPtr_t& constraint);
  // Get numerical constraints of a graph, a state or a transition
  NumericalConstraints_t getNumericalConstraints(const std::string& name);
  // Project a configuration on a state
  std::tuple<bool, Configuration_t, value_type> applyStateConstraints(
      const std::string& nodeName, ConfigurationIn_t input);
  // Project configuration on the leaf of a transition
  std::tuple<bool, Configuration_t, value_type> applyLeafConstraints(
      const std::string& transitionName, ConfigurationIn_t q_rhs,
      ConfigurationIn_t input);
  // Project configuration at intersection of a leaf and the goal state of a
  // trnasition
  std::tuple<bool, Configuration_t, value_type> generateTargetConfig(
      const std::string& transitionName, ConfigurationIn_t q_rhs,
      ConfigurationIn_t input);
};  // class Graph
}  // namespace manipulation
}  // namespace pyhpp
