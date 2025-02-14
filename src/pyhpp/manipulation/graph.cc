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

#include <boost/python.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/state-selector.hh>

#include <../src/pyhpp/manipulation/device.hh>
#include <../src/pyhpp/manipulation/graph.hh>

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

template <typename T>
std::string toStr() {
  return typeid(T).name();
}
template <>
std::string toStr<hpp::manipulation::graph::State>() {
  return "State";
}
template <>
std::string toStr<hpp::manipulation::graph::Edge>() {
  return "Transition";
}
template <>
std::string toStr<hpp::manipulation::graph::Graph>() {
  return "Graph";
}
template <>
std::string toStr<hpp::manipulation::graph::LevelSetEdge>() {
  return "LevelSetTransition";
}
template <>
std::string toStr<hpp::manipulation::graph::WaypointEdge>() {
  return "WaypointTransition";
}

template <typename T>
std::shared_ptr<T> Graph::getComp(const std::string& name) {
  std::shared_ptr<T> comp;
  try {
    auto it(id.find(name));
    if (it == id.end()) {
      std::stringstream ss;
      ss << "\"" << name << "\" is not a " << toStr<T>();
      throw std::logic_error(ss.str().c_str());
    }
    comp = HPP_DYNAMIC_PTR_CAST(T, obj->get(it->second).lock());
  } catch (std::out_of_range& e) {
    throw std::logic_error(e.what());
  }
  if (!comp) {
    std::stringstream ss;
    ss << "\"" << name << "\" is not a " << toStr<T>();
    throw std::logic_error(ss.str().c_str());
  }
  return comp;
}

// Wrapper class to hpp::manipulation::graph::Graph
//
// Boost python does not correctly handle weak pointers. To overcome this limitation,
// we create a wrapper class and bind this class with boost python instead of the original class.
Graph::Graph(const hpp::manipulation::graph::GraphPtr_t& object) : obj(object) {
}
Graph::Graph(const std::string& name, const Device& d, const ProblemPtr_t& problem) :
  obj(hpp::manipulation::graph::Graph::create(name, d.obj, problem)) {
}

std::string Graph::name() const {
  return obj->name();
}

std::size_t Graph::createState(const std::string& nodeName, bool waypoint, int priority) {
  hpp::manipulation::graph::GraphPtr_t graph = obj;
  if (graph->stateSelector()) {
    hpp::manipulation::graph::StatePtr_t state =
      graph->stateSelector()->createState(nodeName, waypoint, priority);
    id[nodeName] = state->id();
    return state->id();
  } else {
    throw std::logic_error("Graph has no state selector.");
  }
}

std::size_t Graph::createTransition(const std::string& nodeFrom, const std::string& nodeTo,
				    const std::string& transitionName, int w,
				    const std::string& isInState) {
  using namespace hpp::manipulation::graph;
  StatePtr_t from = getComp<State>(nodeFrom),
    to = getComp<State>(nodeTo),
    inState = getComp<State>(isInState);

  EdgePtr_t edge = from->linkTo(transitionName, to, w,
                                (State::EdgeFactory)Edge::create);
  edge->state(inState);

  return (std::size_t)edge->id();
}

void Graph::addNumericalConstraint(const std::string& name,
				   const ImplicitPtr_t& constraint) {
  using namespace hpp::manipulation::graph;
  GraphComponentPtr_t component =
      getComp<GraphComponent>(name);
  component->addNumericalConstraint(constraint);
}

hpp::constraints::NumericalConstraints_t Graph::getNumericalConstraints(const std::string& name) {
  using namespace hpp::manipulation::graph;
  GraphComponentPtr_t component =
      getComp<GraphComponent>(name);
  return component->numericalConstraints();
}

std::tuple<bool, Configuration_t, value_type> Graph::applyStateConstraints
  (const std::string& nodeName, ConfigurationIn_t input){
  using namespace hpp::manipulation;
  // Recover state from name
  graph::StatePtr_t state = getComp<graph::State>(nodeName);
  ConstraintSetPtr_t constraint(state->configConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(constraint->apply(output));
  if (hpp::core::ConfigProjectorPtr_t configProjector = constraint->configProjector()) {
    residualError = configProjector->residualError();
  }
  return std::make_tuple(success, output, residualError);
}

std::tuple<bool, Configuration_t, value_type> Graph::applyLeafConstraints
(const std::string& transitionName, ConfigurationIn_t q_rhs, ConfigurationIn_t input)
{
  using namespace hpp::manipulation;
  // Recover state from name
  graph::EdgePtr_t transition = getComp<graph::Edge>(transitionName);
  ConstraintSetPtr_t constraint(transition->pathConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(true);
  if (constraint->configProjector()) {
    constraint->configProjector()->rightHandSideFromConfig(q_rhs);
    success = constraint->apply(output);
    residualError = constraint->configProjector()->residualError();
  } else {
    residualError = std::numeric_limits<value_type>::quiet_NaN();
  }
  return std::make_tuple(success, output, residualError);
}

std::tuple<bool, Configuration_t, value_type> Graph::generateTargetConfig
(const std::string& transitionName, ConfigurationIn_t q_rhs, ConfigurationIn_t input)
{
  using namespace hpp::manipulation;
  // Recover state from name
  graph::EdgePtr_t transition = getComp<graph::Edge>(transitionName);
  ConstraintSetPtr_t constraint(transition->targetConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(true);
  if (constraint->configProjector()) {
    constraint->configProjector()->rightHandSideFromConfig(q_rhs);
    success = constraint->apply(output);
    residualError = constraint->configProjector()->residualError();
  } else {
    residualError = std::numeric_limits<value_type>::quiet_NaN();
  }
  return std::make_tuple(success, output, residualError);
}

using namespace boost::python;

void exposeGraph() {
  class_<Graph>("Graph", init <const std::string&, const Device&, const ProblemPtr_t&> ())
    .def_readonly("name", &Graph::name)
    .def("createState", &Graph::createState)
    .def("createTransition", &Graph::createTransition)
    .def("addNumericalConstraint", &Graph::addNumericalConstraint)
    .def("getNumericalConstraints", &Graph::getNumericalConstraints)
    .def("applyStateConstraints", &Graph::applyStateConstraints)
    .def("applyLeafConstraints", &Graph::applyLeafConstraints)
    .def("generateTargetConfig", &Graph::generateTargetConfig);
}
} // namespace manipulation
} // namespace pyhpp
