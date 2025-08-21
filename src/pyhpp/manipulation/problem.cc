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
#include <boost/python.hpp>
#include <hpp/core/problem.hh>
#include <pyhpp/core/steering-method.hh>
#include <hpp/manipulation/steering-method/graph.hh>

using namespace boost::python;

namespace pyhpp {
namespace manipulation {

Problem::Problem(const PyWDevicePtr_t& robot)
    : pyhpp::core::Problem(hpp::manipulation::Problem::create(robot->obj)) {}

Problem::Problem(const hpp::manipulation::ProblemPtr_t& object)
  : pyhpp::core::Problem(object)  {
}

void Problem::constraintGraph(const PyWGraphPtr_t& graph) {
  asManipulationProblem()->constraintGraph(graph->obj);
}

PyWGraphPtr_t Problem::constraintGraph() const {
  pyhpp::manipulation::PyWGraph* graph = new PyWGraph(asManipulationProblem()->constraintGraph());
  return std::shared_ptr<PyWGraph>(graph);
}

void Problem::checkProblem() const { asManipulationProblem()->checkProblem(); }

void Problem::steeringMethod(const pyhpp::core::PyWSteeringMethodPtr_t& steeringMethod) {
    if (steeringMethod && steeringMethod->obj) {
        auto manipSM = hpp::manipulation::steeringMethod::Graph::create(
            obj);
        obj->steeringMethod(steeringMethod->obj);
    }
}

pyhpp::core::PyWSteeringMethodPtr_t Problem::steeringMethod() const {
    auto steeringMethod = new pyhpp::core::SteeringMethod();
    steeringMethod->obj = asManipulationProblem()->steeringMethod();
    return std::shared_ptr<pyhpp::core::SteeringMethod>(steeringMethod);
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
  boost::python::import("pyhpp.core");
  class_<Problem, bases<pyhpp::core::Problem>>("Problem", init<const PyWDevicePtr_t&>())
      .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(Problem, constraintGraph,
                                            PyWGraphPtr_t)
      .PYHPP_DEFINE_METHOD(Problem, checkProblem)
      .def("steeringMethod", 
            static_cast<pyhpp::core::PyWSteeringMethodPtr_t(Problem::*)() const>(&Problem::steeringMethod))
      .def("steeringMethod",
            static_cast<void(Problem::*)(const pyhpp::core::PyWSteeringMethodPtr_t&)>(&Problem::steeringMethod))
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
