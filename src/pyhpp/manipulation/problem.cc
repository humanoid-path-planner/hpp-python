//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
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
#include <../src/pyhpp/manipulation/steering-method.hh>
#include <boost/python.hpp>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/manipulation/steering-method/end-effector-trajectory.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <pyhpp/core/steering-method.hh>

// DocNamespace(hpp::manipulation)

namespace {

const char* DOC_CONSTRAINTGRAPH_GET = "Get the graph of constraints.";
const char* DOC_CONSTRAINTGRAPH_SET = "Set the graph of constraints.";

}  // namespace

using namespace boost::python;

namespace pyhpp {
namespace manipulation {

Problem::Problem(const PyWDevicePtr_t& robot)
    : pyhpp::core::Problem(
          hpp::manipulation::Problem::create(robot->asManipulationDevice())) {}

Problem::Problem(const hpp::manipulation::ProblemPtr_t& object)
    : pyhpp::core::Problem(object) {}

void Problem::constraintGraph(const PyWGraphPtr_t& graph) {
  asManipulationProblem()->constraintGraph(graph->obj);
  graph_ = graph;
}

PyWGraphPtr_t Problem::constraintGraph() const { return graph_; }

void Problem::checkProblem() const { asManipulationProblem()->checkProblem(); }

void Problem::steeringMethod(
    const pyhpp::core::PyWSteeringMethodPtr_t& steeringMethod) {
  auto manipulationSteeringMethod = HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::SteeringMethod, steeringMethod->obj);
  auto endEffectorTrajectorySteeringMethod = HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::steeringMethod::EndEffectorTrajectory,
      steeringMethod->obj);
  if (manipulationSteeringMethod || endEffectorTrajectorySteeringMethod) {
    obj->steeringMethod(steeringMethod->obj);
    return;
  }

  manipulationSteeringMethod = HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::SteeringMethod, obj->steeringMethod());
  if (!manipulationSteeringMethod) {
    manipulationSteeringMethod =
        hpp::manipulation::steeringMethod::Graph::create(obj);
    obj->steeringMethod(manipulationSteeringMethod);
  }
  manipulationSteeringMethod->innerSteeringMethod(steeringMethod->obj);
}

void Problem::fullSteeringMethod(
    const pyhpp::core::PyWSteeringMethodPtr_t& steeringMethod) {
  obj->steeringMethod(steeringMethod->obj);
}

pyhpp::core::PyWSteeringMethodPtr_t Problem::steeringMethod() const {
  hpp::manipulation::steeringMethod::GraphPtr_t gsm = HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::steeringMethod::Graph, obj->steeringMethod());
  if (!gsm) {
    pyhpp::core::SteeringMethod* sm =
        new pyhpp::core::SteeringMethod(obj->steeringMethod());
    return std::shared_ptr<pyhpp::core::SteeringMethod>(sm);
  }
  pyhpp::core::SteeringMethod* sm =
      new pyhpp::core::SteeringMethod(gsm->innerSteeringMethod());
  return std::shared_ptr<pyhpp::core::SteeringMethod>(sm);
}

// PathValidationPtr_t Problem::pathValidation() const {
//     return obj->pathValidation();
// }

// void Problem::pathValidation(const PathValidationPtr_t &pathValidation) {
//     obj->pathValidation(pathValidation);
// }

// SteeringMethodPtr_t Problem::manipulationSteeringMethod() const {
//     return obj->manipulationSteeringMethod();
// }

// PathValidationPtr_t Problem::pathValidationFactory() const {
//     return obj->pathValidationFactory();
// }

// void Problem::setPathValidationFactory(const core::PathValidationBuilder_t
// &factory, const value_type &tol) {
//     obj->setPathValidationFactory(factory, tol);
// }

// static void declareParameter(const ParameterDescription &desc) {
//     Problem::declareParameter(desc);
// }

// static const Container<ParameterDescription> & parameterDescriptions() {
//     return Problem::parameterDescriptions();
// }

// static const ParameterDescription & parameterDescription(const std::string
// &name) {
//     return Problem::parameterDescription(name);
// }

void exposeProblem() {
  // DocClass(Problem)
  class_<Problem, bases<pyhpp::core::Problem>>("Problem", DocClassDoc(),
                                               init<const PyWDevicePtr_t&>())
      .def("constraintGraph",
           static_cast<PyWGraphPtr_t (Problem::*)() const>(
               &Problem::constraintGraph),
           DOC_CONSTRAINTGRAPH_GET)
      .def("constraintGraph",
           static_cast<void (Problem::*)(const PyWGraphPtr_t&)>(
               &Problem::constraintGraph),
           DOC_CONSTRAINTGRAPH_SET)
      .def("checkProblem", &Problem::checkProblem, DocClassMethod(checkProblem))
      .def(
          "steeringMethod",
          static_cast<pyhpp::core::PyWSteeringMethodPtr_t (Problem::*)() const>(
              &Problem::steeringMethod),
          "Get the inner steering method (unwrapped from the graph steering "
          "method if applicable).")
      .def("steeringMethod",
           static_cast<void (Problem::*)(
               const pyhpp::core::PyWSteeringMethodPtr_t&)>(
               &Problem::steeringMethod),
           "Set the steering method.")

      .def("fullSteeringMethod", &Problem::fullSteeringMethod,
           "Set the problem steering method directly. Unlike steeringMethod, "
           "this does not wrap the given steering method in a manipulation "
           "graph steering method.")
      // .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(Problem, pathValidation,
      // PathValidationPtr_t) .PYHPP_DEFINE_METHOD(Problem,
      // manipulationSteeringMethod) .PYHPP_DEFINE_METHOD(Problem,
      // pathValidationFactory) .PYHPP_DEFINE_METHOD(Problem,
      // setPathValidationFactory) .PYHPP_DEFINE_METHOD_STATIC(Problem,
      // declareParameter) .PYHPP_DEFINE_METHOD_STATIC(Problem,
      // parameterDescriptions) .PYHPP_DEFINE_METHOD_STATIC(Problem,
      // parameterDescription)
      ;
}
}  // namespace manipulation
}  // namespace pyhpp
