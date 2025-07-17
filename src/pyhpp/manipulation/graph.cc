//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux, Paul Sardin
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

#include <../src/pyhpp/manipulation/device.hh>
#include <../src/pyhpp/manipulation/graph.hh>
#include <../src/pyhpp/manipulation/problem.hh>

#include <boost/python.hpp>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/guided-state-selector.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <pinocchio/spatial/se3.hpp>

namespace {

    const char* DOC_CREATESTATE = 
        "Create one or several states. "
        "The order is important - the first should be the most restrictive one as a configuration "
        "will be in the first state for which the constraints are satisfied.";

    const char* DOC_CREATETRANSITION = 
        "Create a transition. "
        "The weights define the probability of selecting a transition among all the "
        "outgoing transitions of a state. The probability of a transition is "
        "w_i / sum(w_j), where each w_j corresponds to an outgoing transition from a given state. "
        "To have a transition that cannot be selected by the M-RRT algorithm but is still "
        "acceptable, set its weight to zero.";

    const char* DOC_SETCONTAININGNODE = 
        "Set in which state a transition is. "
        "Paths satisfying the transition constraints satisfy the state constraints.";

    const char* DOC_GETCONTAININGNODE = 
        "Get the name of the state in which a transition is. "
        "Paths satisfying the transition constraints satisfy the state constraints.";

    const char* DOC_SETSHORT = 
        "Set that a transition is short. "
        "When a transition is tagged as short, extension along this transition is "
        "done differently in RRT-like algorithms. Instead of projecting "
        "a random configuration in the destination state, the "
        "configuration to extend itself is projected in the destination "
        "state. This makes the rate of success higher.";

    const char* DOC_ISSHORT = 
        "Check if a transition is short.";

    const char* DOC_CREATEWAYPOINTTRANSITION = 
        "Create a WaypointTransition. "
        "See documentation of class hpp::manipulation::graph::WaypointEdge "
        "for more information.";

    const char* DOC_CREATELEVELSETTRANSITION = 
        "Create a LevelSetTransition. "
        "See documentation of class hpp::manipulation::graph::LevelSetEdge "
        "for more information.";

    const char* DOC_GETNODESCONNECTEDBYTRANSITION = 
        "Get the names of the states connected by a transition.";

    const char* DOC_GETWEIGHT = 
        "Get weight of a transition.";

    const char* DOC_SETWEIGHT = 
        "Set weight of a transition. "
        "You cannot set weight for waypoint transitions.";

    const char* DOC_GETSTATE = 
        "Get the name of the state corresponding to the configuration.";

    const char* DOC_ADDNUMERICALCONSTRAINT = 
        "Add a numerical constraint to a state.";

    const char* DOC_ADDNUMERICALCONSTRAINTS = 
        "Add numerical constraints to a state.";

    const char* DOC_ADDNUMERICALCONSTRAINTSFORPATH = 
        "Add numerical constraints for path to a state.";

    const char* DOC_GETNUMERICALCONSTRAINTS = 
        "Get numerical constraints of a state.";

    const char* DOC_RESETCONSTRAINTS = 
        "Reset constraints of a state.";

    const char* DOC_REGISTERCONSTRAINTS = 
        "Register constraints in the graph.";

    const char* DOC_GETCONFIGERRORFORSTATE = 
        "Get error of a config with respect to a state constraint. "
        "Returns whether the configuration belongs to the state. "
        "Calls core::ConstraintSet::isSatisfied for the state constraints.";

    const char* DOC_GETCONFIGERRORFORTRANSITION = 
        "Get error of a config with respect to a transition constraint. "
        "Returns whether the configuration belongs to the transition. "
        "Calls core::ConfigProjector::rightHandSideFromConfig with "
        "the input configuration and then core::ConstraintSet::isSatisfied "
        "on the transition constraints.";

    const char* DOC_GETCONFIGERRORFORTRANSITIONLEAF = 
        "Get error of a config with respect to a transition foliation leaf. "
        "Returns whether config can be the end point of a path of the transition "
        "starting at leafConfig.";

    const char* DOC_GETCONFIGERRORFORTRANSITIONTARGET = 
        "Get error of a config with respect to the target of a transition foliation leaf. "
        "Returns whether config can be the end point of a path of the transition "
        "starting at leafConfig.";

    const char* DOC_APPLYSTATECONSTRAINTS = 
        "Apply constraints to a configuration. "
        "Returns ConstraintResult with success flag, output configuration, and error norm.";

    const char* DOC_APPLYLEAFCONSTRAINTS = 
        "Apply transition constraints to a configuration. "
        "Returns ConstraintResult with success flag, output configuration, and error norm. "
        "If success, the output configuration is reachable from q_rhs along the transition.";

    const char* DOC_GENERATETARGETCONFIG = 
        "Generate configuration in destination state on a given leaf. "
        "Returns ConstraintResult with success flag, output configuration, and error norm. "
        "Computes a configuration in the destination state of the transition, "
        "reachable from q_rhs.";

    const char* DOC_ADDLEVELSETFOLIATION = 
        "Add the numerical constraints to a LevelSetTransition that create the foliation.";

    const char* DOC_GETSECURITYMARGINMATRIXFORTRANSITION = 
        "Get security margin matrix for a transition as list of lists.";

    const char* DOC_SETSECURITYMARGINFORTRANSITION = 
        "Set collision security margin for a pair of joints along a transition.";

    const char* DOC_GETRELATIVEMOTIONMATRIX = 
        "Get relative motion matrix for a transition as list of lists.";

    const char* DOC_REMOVECOLLISIONPAIRFROMTRANSITION = 
        "Remove collision pairs from a transition.";

    const char* DOC_CREATESUBGRAPH = 
        "Create a subgraph with guided state selection.";

    const char* DOC_SETTARGETNODELIST = 
        "Set the target state list for guided state selection.";

    const char* DOC_DISPLAYSTATECONSTRAINTS = 
        "Print set of constraints relative to a state in a string.";

    const char* DOC_DISPLAYTRANSITIONCONSTRAINTS = 
        "Print set of constraints relative to a transition in a string.";

    const char* DOC_DISPLAYTRANSITIONTARGETCONSTRAINTS = 
        "Print set of constraints relative to a transition in a string.";

    const char* DOC_DISPLAY = 
        "Display the current graph. "
        "The graph is printed in DOT format.";

    const char* DOC_INITIALIZE = 
        "Initialize the graph. "
        "Performs final initialization of the constraint graph.";

}

namespace pyhpp {
namespace manipulation {

// =============================================================================
// Utility functions
// =============================================================================

std::vector<std::vector<int>> matrixToVectorVector(
    const Eigen::Ref<const Eigen::Matrix<double, -1, -1>>& input) {
  std::vector<std::vector<int>> result;
  result.reserve(input.rows());

  for (int i = 0; i < input.rows(); ++i) {
    std::vector<int> row;
    row.reserve(input.cols());
    for (int j = 0; j < input.cols(); ++j) {
      row.push_back(static_cast<int>(input(i, j)));
    }
    result.push_back(std::move(row));
  }
  return result;
}

// =============================================================================
// Wrapper class constructors
// =============================================================================

PyWState::PyWState(const StatePtr_t& state) : obj(state) {}

PyWEdge::PyWEdge(const EdgePtr_t& edge) : obj(edge) {}

PyWGraph::PyWGraph(const hpp::manipulation::graph::GraphPtr_t& object)
    : obj(object) {}

PyWGraph::PyWGraph(const std::string& name, const PyWDevicePtr_t& d,
                   const PyWProblemPtr_t& problem)
    : obj(hpp::manipulation::graph::Graph::create(name, d->obj, problem->obj)) {}

// =============================================================================
// Configuration methods
// =============================================================================

void PyWGraph::maxIterations(size_type iterations) {
  obj->maxIterations(iterations);
}

size_type PyWGraph::maxIterations() const {
  return obj->maxIterations();
}

void PyWGraph::errorThreshold(const value_type& threshold) {
  obj->errorThreshold(threshold);
}

value_type PyWGraph::errorThreshold() const {
  return obj->errorThreshold();
}

// =============================================================================
// Graph construction
// =============================================================================

PyWStatePtr_t PyWGraph::createState(const std::string& nodeName, bool waypoint,
                                     int priority) {
  if (obj->stateSelector()) {
    hpp::manipulation::graph::StatePtr_t state =
        obj->stateSelector()->createState(nodeName, waypoint, priority);
    return std::shared_ptr<PyWState>(new PyWState(state));
  } else {
    throw std::logic_error("Graph has no state selector.");
  }
}

PyWEdgePtr_t PyWGraph::createTransition(PyWStatePtr_t nodeFrom, PyWStatePtr_t nodeTo,
                                  const std::string& transitionName, int w,
                                  PyWStatePtr_t isInState) {
  using namespace hpp::manipulation::graph;
  EdgePtr_t edge = nodeFrom->obj->linkTo(
      transitionName, nodeTo->obj, w, (State::EdgeFactory)Edge::create);
  edge->state(isInState->obj);
  return std::shared_ptr<PyWEdge>(new PyWEdge(edge));
}

PyWEdgePtr_t PyWGraph::createWaypointTransition(PyWStatePtr_t nodeFrom,
                                           PyWStatePtr_t nodeTo,
                                           const std::string& edgeName, int nb,
                                           int w, PyWStatePtr_t isInState) {
  try {
    using namespace hpp::manipulation::graph;
    EdgePtr_t edge_pc = nodeFrom->obj->linkTo(
        edgeName, nodeTo->obj, w, (State::EdgeFactory)WaypointEdge::create);
    edge_pc->state(isInState->obj);

    auto waypoint_edge = HPP_DYNAMIC_PTR_CAST(WaypointEdge, edge_pc);
    waypoint_edge->nbWaypoints(nb);

    return std::shared_ptr<PyWEdge>(new PyWEdge(edge_pc));
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

PyWEdgePtr_t PyWGraph::createLevelSetTransition(PyWStatePtr_t nodeFrom,
                                           PyWStatePtr_t nodeTo,
                                           const std::string& edgeName, int w,
                                           PyWStatePtr_t isInState) {
  try {
    using namespace hpp::manipulation::graph;
    EdgePtr_t edge = nodeFrom->obj->linkTo(
        edgeName, nodeTo->obj, w, (State::EdgeFactory)LevelSetEdge::create);
    edge->state(isInState->obj);
    return std::shared_ptr<PyWEdge>(new PyWEdge(edge));
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// Edge/State management
// =============================================================================

void PyWGraph::setContainingNode(PyWEdgePtr_t edge, PyWStatePtr_t state) {
  try {
    edge->obj->state(state->obj);
  } catch (std::exception& err) {
    throw std::logic_error(err.what());
  }
}

std::string PyWGraph::getContainingNode(PyWEdgePtr_t edge) {
  try {
    return edge->obj->state()->name();
  } catch (std::exception& err) {
    throw std::logic_error(err.what());
  }
}

void PyWGraph::setShort(PyWEdgePtr_t edge, bool isShort) {
  try {
    edge->obj->setShort(isShort);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

bool PyWGraph::isShort(PyWEdgePtr_t edge) {
  try {
    return edge->obj->isShort();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::getNodesConnectedByTransition(PyWEdgePtr_t edge,
                                        std::string& nodeFrom,
                                        std::string& nodeTo) {
  try {
    nodeFrom = edge->obj->stateFrom()->name();
    nodeTo = edge->obj->stateTo()->name();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::setWeight(PyWEdgePtr_t edge, int weight) {
  try {
    edge->obj->stateFrom()->updateWeight(edge->obj, weight);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

size_t PyWGraph::getWeight(PyWEdgePtr_t edge) {
  try {
    return edge->obj->stateFrom()->getWeight(edge->obj);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// State queries
// =============================================================================

std::string PyWGraph::getState(ConfigurationIn_t input) {
  try {
    hpp::manipulation::graph::StatePtr_t state = obj->getState(input);
    return state->name();
  } catch (std::exception& e) {
    throw std::logic_error(e.what());
  }
}

// =============================================================================
// Constraint management
// =============================================================================

void PyWGraph::addNumericalConstraint(PyWStatePtr_t node,
                                       const ImplicitPtr_t& constraint) {
  node->obj->addNumericalConstraint(constraint);
}

void PyWGraph::addNumericalConstraints(
    PyWStatePtr_t component, const boost::python::list& py_constraints) {
  auto constraints =
      extract_vector<std::shared_ptr<hpp::constraints::Implicit>>(
          py_constraints);
  if (constraints.size() > 0) {
    try {
      for (size_t i = 0; i < constraints.size(); ++i) {
        component->obj->addNumericalConstraint(constraints[i]);
      }
    } catch (std::exception& err) {
      throw std::logic_error(err.what());
    }
  }
}

void PyWGraph::addNumericalConstraintsForPath(
    PyWStatePtr_t component, const boost::python::list& py_constraints) {
  auto constraints =
      extract_vector<std::shared_ptr<hpp::constraints::Implicit>>(
          py_constraints);
  if (constraints.size() > 0) {
    try {
      for (size_t i = 0; i < constraints.size(); ++i) {
        component->obj->addNumericalConstraintForPath(constraints[i]);
      }
    } catch (std::exception& err) {
      throw std::logic_error(err.what());
    }
  }
}

boost::python::list PyWGraph::getNumericalConstraints(PyWStatePtr_t component) {
  return to_python_list(component->obj->numericalConstraints());
}

void PyWGraph::resetConstraints(PyWStatePtr_t component) {
  component->obj->resetNumericalConstraints();
}

void PyWGraph::registerConstraints(const ImplicitPtr_t& constraint,
                                    const ImplicitPtr_t& complement,
                                    const ImplicitPtr_t& both) {
  try {
    obj->registerConstraints(constraint, complement, both);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// Configuration error checking
// =============================================================================

bool PyWGraph::getConfigErrorForState(PyWStatePtr_t component,
                                       ConfigurationIn_t input,
                                       hpp::core::vector_t& error) {
  try {
    return obj->getConfigErrorForState(input, component->obj, error);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

bool PyWGraph::getConfigErrorForTransition(PyWEdgePtr_t edge,
                                      ConfigurationIn_t input,
                                      hpp::core::vector_t& error) {
  try {
    // Check if steering method is properly initialized
    if (!edge->obj->parentGraph()->problem()->manipulationSteeringMethod() ||
        !edge->obj->parentGraph()
             ->problem()
             ->manipulationSteeringMethod()
             ->innerSteeringMethod()) {
      throw std::logic_error("Could not initialize the steering method.");
    }
    return obj->getConfigErrorForEdge(input, edge->obj, error);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

bool PyWGraph::getConfigErrorForTransitionLeaf(ConfigurationIn_t leafConfig,
                                          ConfigurationIn_t config,
                                          const PyWEdgePtr_t& edge,
                                          hpp::core::vector_t& error) const {
  return obj->getConfigErrorForEdgeLeaf(leafConfig, config, edge->obj, error);
}

bool PyWGraph::getConfigErrorForTransitionTarget(ConfigurationIn_t leafConfig,
                                            ConfigurationIn_t config,
                                            const PyWEdgePtr_t& edge,
                                            hpp::core::vector_t& error) const {
  return obj->getConfigErrorForEdgeTarget(leafConfig, config, edge->obj, error);
}

// =============================================================================
// Constraint application
// =============================================================================

ConstraintResult PyWGraph::applyStateConstraints(PyWStatePtr_t state,
                                                  ConfigurationIn_t input) {
  using namespace hpp::manipulation;
  ConstraintSetPtr_t constraint(state->obj->configConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(constraint->apply(output));

  if (hpp::core::ConfigProjectorPtr_t configProjector =
          constraint->configProjector()) {
    residualError = configProjector->residualError();
  }
  return ConstraintResult(success, output, residualError);
}

ConstraintResult PyWGraph::applyLeafConstraints(PyWEdgePtr_t transition,
                                                 ConfigurationIn_t q_rhs,
                                                 ConfigurationIn_t input) {
  using namespace hpp::manipulation;
  ConstraintSetPtr_t constraint(transition->obj->pathConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(true);

  if (constraint->configProjector()) {
    constraint->configProjector()->rightHandSideFromConfig(q_rhs);
    success = constraint->apply(output);
    residualError = constraint->configProjector()->residualError();
  }
  return ConstraintResult(success, output, residualError);
}

ConstraintResult PyWGraph::generateTargetConfig(PyWEdgePtr_t transition,
                                                 ConfigurationIn_t q_rhs,
                                                 ConfigurationIn_t input) {
  using namespace hpp::manipulation;
  ConstraintSetPtr_t constraint(transition->obj->targetConstraint());
  value_type residualError(std::numeric_limits<value_type>::quiet_NaN());
  Configuration_t output(input);
  bool success(true);

  if (constraint->configProjector()) {
    constraint->configProjector()->rightHandSideFromConfig(q_rhs);
    success = constraint->apply(output);
    residualError = constraint->configProjector()->residualError();
  }
  return ConstraintResult(success, output, residualError);
}

// =============================================================================
// Level set edges
// =============================================================================

void PyWGraph::addLevelSetFoliation(PyWEdgePtr_t edge,
                                     const boost::python::list& condNC,
                                     const boost::python::list& paramNC) {
  try {
    using namespace hpp::manipulation::graph;
    auto levelSetEdge = HPP_DYNAMIC_PTR_CAST(LevelSetEdge, edge->obj);
    if (!levelSetEdge) {
      throw std::logic_error("Edge is not a LevelSetEdge");
    }

    auto condConstraints = extract_vector<ImplicitPtr_t>(condNC);
    auto paramConstraints = extract_vector<ImplicitPtr_t>(paramNC);

    for (const auto& constraint : condConstraints) {
      levelSetEdge->insertConditionConstraint(constraint);
    }
    for (const auto& constraint : paramConstraints) {
      levelSetEdge->insertParamConstraint(constraint);
    }
  } catch (std::exception& err) {
    throw std::logic_error(err.what());
  }
}

// =============================================================================
// Security margins and collision
// =============================================================================

boost::python::list PyWGraph::getSecurityMarginMatrixForTransition(
    PyWEdgePtr_t edge) {
  try {
    std::vector<std::vector<int>> matrix =
        matrixToVectorVector(edge->obj->securityMargins());

    boost::python::list result;
    for (const auto& row : matrix) {
      result.append(to_python_list(row));
    }
    return result;
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::setSecurityMarginForTransition(PyWEdgePtr_t edge, const char* joint1,
                                         const char* joint2, double margin) {
  using namespace hpp::manipulation;
  try {
    JointPtr_t j1(obj->robot()->getJointByName(joint1));
    JointPtr_t j2(obj->robot()->getJointByName(joint2));
    
    size_type i1 = j1 ? j1->index() : 0;
    size_type i2 = j2 ? j2->index() : 0;
    
    edge->obj->securityMarginForPair(i1, i2, margin);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

boost::python::list PyWGraph::getRelativeMotionMatrix(PyWEdgePtr_t edge) {
  try {
    Eigen::MatrixXi intMatrix = edge->obj->relativeMotion().cast<int>();

    std::vector<std::vector<int>> matrix;
    matrix.reserve(intMatrix.rows());

    for (int i = 0; i < intMatrix.rows(); ++i) {
      std::vector<int> row;
      row.reserve(intMatrix.cols());
      for (int j = 0; j < intMatrix.cols(); ++j) {
        row.push_back(intMatrix(i, j));
      }
      matrix.push_back(std::move(row));
    }

    boost::python::list result;
    for (const auto& row : matrix) {
      result.append(to_python_list(row));
    }
    return result;
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::removeCollisionPairFromTransition(PyWEdgePtr_t edge,
                                            const char* joint1,
                                            const char* joint2) {
  try {
    using hpp::core::RelativeMotion;
    RelativeMotion::matrix_type m(edge->obj->relativeMotion());
    DevicePtr_t robot = obj->robot();

    auto j1 = robot->getJointByName(joint1);
    auto j2 = robot->getJointByName(joint2);

    hpp::manipulation::JointIndex i1 = j1->index();
    hpp::manipulation::JointIndex i2 = j2->index();

    m(i1, i2) = m(i2, i1) = RelativeMotion::Constrained;
    edge->obj->relativeMotion(m);
  } catch (std::exception& err) {
    throw std::logic_error(err.what());
  }
}

// =============================================================================
// Subgraph management
// =============================================================================

void PyWGraph::createSubGraph(const char* subgraphName,
                               hpp::core::RoadmapPtr_t roadmap) {
  using namespace hpp::manipulation;
  try {
    graph::GuidedStateSelectorPtr_t ns =
        graph::GuidedStateSelector::create(subgraphName, roadmap);
    obj->stateSelector(ns);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::setTargetNodeList(const boost::python::list& py_nodes) {
  using namespace hpp::manipulation;
  auto nodes = extract_vector<PyWStatePtr_t>(py_nodes);
  graph::GuidedStateSelectorPtr_t ns =
      HPP_DYNAMIC_PTR_CAST(graph::GuidedStateSelector, obj->stateSelector());
  if (!ns) {
    throw std::logic_error(
        "The state selector is not of type GuidedStateSelector.");
  }
  
  try {
    graph::States_t nl;
    for (size_t i = 0; i < nodes.size(); ++i) {
      nl.push_back(nodes[i]->obj);
    }
    ns->setStateList(nl);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// Display and debugging
// =============================================================================

std::string PyWGraph::displayStateConstraints(PyWStatePtr_t state) {
  using namespace hpp::manipulation;
  try {
    ConstraintSetPtr_t cs(obj->configConstraint(state->obj));
    std::ostringstream oss;
    oss << (*cs);
    return oss.str();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

std::string PyWGraph::displayTransitionConstraints(PyWEdgePtr_t edge) {
  using namespace hpp::manipulation;
  try {
    ConstraintSetPtr_t cs(obj->pathConstraint(edge->obj));
    std::ostringstream oss;
    oss << (*cs);
    return oss.str();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

std::string PyWGraph::displayTransitionTargetConstraints(PyWEdgePtr_t edge) {
  using namespace hpp::manipulation;
  try {
    ConstraintSetPtr_t cs(obj->targetConstraint(edge->obj));
    std::ostringstream oss;
    oss << (*cs);
    return oss.str();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

void PyWGraph::display(const char* filename) {
  try {
    std::ofstream dotfile;
    dotfile.open(filename);
    obj->dotPrint(dotfile);
    dotfile.close();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// Initialization
// =============================================================================

void PyWGraph::initialize() {
  try {
    obj->initialize();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

// =============================================================================
// Boost.Python bindings
// =============================================================================

using namespace boost::python;

void exposeGraph() {
  class_<PyWState, PyWStatePtr_t>("State", no_init);
  
  class_<PyWEdge, PyWEdgePtr_t>("Transition", no_init);

  class_<PyWGraph>("Graph", init<const std::string&, const PyWDevicePtr_t&,
                                const PyWProblemPtr_t&>())

      // Configuration methods
      .PYHPP_DEFINE_GETTER_SETTER(PyWGraph, maxIterations, hpp::manipulation::size_type)
      .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(PyWGraph, errorThreshold, hpp::manipulation::value_type)

      // Graph construction
      .PYHPP_DEFINE_METHOD1(PyWGraph, createState, DOC_CREATESTATE)
      .PYHPP_DEFINE_METHOD1(PyWGraph, createTransition, DOC_CREATETRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, createWaypointTransition, DOC_CREATEWAYPOINTTRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, createLevelSetTransition, DOC_CREATELEVELSETTRANSITION)

      // Transition/State management
      .PYHPP_DEFINE_METHOD1(PyWGraph, setContainingNode, DOC_SETCONTAININGNODE)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getContainingNode, DOC_GETCONTAININGNODE)
      .PYHPP_DEFINE_METHOD1(PyWGraph, setShort, DOC_SETSHORT)
      .PYHPP_DEFINE_METHOD1(PyWGraph, isShort, DOC_ISSHORT)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getNodesConnectedByTransition, DOC_GETNODESCONNECTEDBYTRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, setWeight, DOC_SETWEIGHT)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getWeight, DOC_GETWEIGHT)

      // State queries
      .PYHPP_DEFINE_METHOD1(PyWGraph, getState, DOC_GETSTATE)

      // Constraint management
      .PYHPP_DEFINE_METHOD1(PyWGraph, addNumericalConstraint, DOC_ADDNUMERICALCONSTRAINT)
      .PYHPP_DEFINE_METHOD1(PyWGraph, addNumericalConstraints, DOC_ADDNUMERICALCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, addNumericalConstraintsForPath, DOC_ADDNUMERICALCONSTRAINTSFORPATH)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getNumericalConstraints, DOC_GETNUMERICALCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, resetConstraints, DOC_RESETCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, registerConstraints, DOC_REGISTERCONSTRAINTS)

      // Configuration error checking
      .PYHPP_DEFINE_METHOD1(PyWGraph, getConfigErrorForState, DOC_GETCONFIGERRORFORSTATE)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getConfigErrorForTransition, DOC_GETCONFIGERRORFORTRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getConfigErrorForTransitionLeaf, DOC_GETCONFIGERRORFORTRANSITIONLEAF)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getConfigErrorForTransitionTarget, DOC_GETCONFIGERRORFORTRANSITIONTARGET)

      // Constraint application
      .PYHPP_DEFINE_METHOD1(PyWGraph, applyStateConstraints, DOC_APPLYSTATECONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, applyLeafConstraints, DOC_APPLYLEAFCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, generateTargetConfig, DOC_GENERATETARGETCONFIG)

      // Level set transitions
      .PYHPP_DEFINE_METHOD1(PyWGraph, addLevelSetFoliation, DOC_ADDLEVELSETFOLIATION)

      // Security margins and collision
      .PYHPP_DEFINE_METHOD1(PyWGraph, getSecurityMarginMatrixForTransition, DOC_GETSECURITYMARGINMATRIXFORTRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, setSecurityMarginForTransition, DOC_SETSECURITYMARGINFORTRANSITION)
      .PYHPP_DEFINE_METHOD1(PyWGraph, getRelativeMotionMatrix, DOC_GETRELATIVEMOTIONMATRIX)
      .PYHPP_DEFINE_METHOD1(PyWGraph, removeCollisionPairFromTransition, DOC_REMOVECOLLISIONPAIRFROMTRANSITION)

      // Subgraph management
      .PYHPP_DEFINE_METHOD1(PyWGraph, createSubGraph, DOC_CREATESUBGRAPH)
      .PYHPP_DEFINE_METHOD1(PyWGraph, setTargetNodeList, DOC_SETTARGETNODELIST)

      // Display and debugging
      .PYHPP_DEFINE_METHOD1(PyWGraph, displayStateConstraints, DOC_DISPLAYSTATECONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, displayTransitionConstraints, DOC_DISPLAYTRANSITIONCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, displayTransitionTargetConstraints, DOC_DISPLAYTRANSITIONTARGETCONSTRAINTS)
      .PYHPP_DEFINE_METHOD1(PyWGraph, display, DOC_DISPLAY)

      // Initialization
      .PYHPP_DEFINE_METHOD1(PyWGraph, initialize, DOC_INITIALIZE);

  class_<ConstraintResult>("ConstraintResult")
      .def_readwrite("success", &ConstraintResult::success)
      .def_readwrite("configuration", &ConstraintResult::configuration)
      .def_readwrite("error", &ConstraintResult::error);
}

}  // namespace manipulation
}  // namespace pyhpp