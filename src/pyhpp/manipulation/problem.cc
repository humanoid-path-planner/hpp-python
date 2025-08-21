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

using namespace boost::python;

namespace pyhpp {
namespace manipulation {

Problem::Problem(const PyWDevicePtr_t& robot)
    : obj(hpp::manipulation::Problem::create(robot->obj)) {}

Problem::Problem(const hpp::manipulation::ProblemPtr_t& object) {
  obj = object;
}

void Problem::constraintGraph(const PyWGraphPtr_t& graph) {
  obj->constraintGraph(graph->obj);
}

PyWGraphPtr_t Problem::constraintGraph() const {
  pyhpp::manipulation::PyWGraph* graph = new PyWGraph(obj->constraintGraph());
  return std::shared_ptr<PyWGraph>(graph);
}

void Problem::checkProblem() const { obj->checkProblem(); }


ConfigurationShooterPtr_t Problem::configurationShooter() const {
    return obj->configurationShooter();
}

SteeringMethodPtr_t Problem::steeringMethod() const {
  return obj->steeringMethod();
}

const ConfigValidationsPtr_t& Problem::configValidation() const {
  return obj->configValidations();
}

PathValidationPtr_t Problem::pathValidation() const {
  return obj->pathValidation();
}

PathProjectorPtr_t Problem::pathProjector() const {
  return obj->pathProjector();
}

DistancePtr_t Problem::distance() const { return obj->distance(); }

const ProblemTargetPtr_t& Problem::target() const { return obj->target(); }

void Problem::configurationShooter(const ConfigurationShooterPtr_t& cs) {
    obj->configurationShooter(cs);
}


void Problem::steeringMethod(const SteeringMethodPtr_t& sm) {
    obj->steeringMethod(sm);
}

void Problem::configValidation(const ConfigValidationsPtr_t& cv) {
    obj->configValidation(cv);
}

void Problem::pathValidation(const PathValidationPtr_t& pv) {
    obj->pathValidation(pv);
}

void Problem::pathProjector(const PathProjectorPtr_t& pp) {
    obj->pathProjector(pp);
}

void Problem::distance(const DistancePtr_t& d) {
    obj->distance(d);
}

void Problem::target(const ProblemTargetPtr_t& t) {
    obj->target(t);
}


void Problem::initConfig(ConfigurationIn_t inConfig) {
  obj->initConfig(inConfig);
}

void Problem::addGoalConfig(ConfigurationIn_t config) {
  obj->addGoalConfig(config);
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

typedef SteeringMethodPtr_t (Problem::*GetSteeringMethod)() const;
typedef void (Problem::*SetSteeringMethod)(const SteeringMethodPtr_t&);

typedef const ConfigValidationsPtr_t& (Problem::*GetConfigValidation)() const;
typedef void (Problem::*SetConfigValidation)(const ConfigValidationsPtr_t&);

typedef PathValidationPtr_t (Problem::*GetPathValidation)() const;
typedef void (Problem::*SetPathValidation)(const PathValidationPtr_t&);

typedef PathProjectorPtr_t (Problem::*GetPathProjector)() const;
typedef void (Problem::*SetPathProjector)(const PathProjectorPtr_t&);

typedef DistancePtr_t (Problem::*GetDistance)() const;
typedef void (Problem::*SetDistance)(const DistancePtr_t&);

typedef const ProblemTargetPtr_t& (Problem::*GetTarget)() const;
typedef void (Problem::*SetTarget)(const ProblemTargetPtr_t&);

typedef ConfigurationShooterPtr_t (Problem::*GetConfigurationShooter)() const;
typedef void (Problem::*SetConfigurationShooter)(const ConfigurationShooterPtr_t&);


void exposeProblem() {
  class_<Problem>("Problem", init<const PyWDevicePtr_t&>())
      .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(Problem, constraintGraph,
                                            PyWGraphPtr_t)
      .PYHPP_DEFINE_METHOD(Problem, checkProblem)
      .PYHPP_DEFINE_METHOD(Problem, initConfig)
      .PYHPP_DEFINE_METHOD(Problem, addGoalConfig)
      .def("steeringMethod", 
          static_cast<GetSteeringMethod>(&Problem::steeringMethod))
      .def("steeringMethod", 
          static_cast<SetSteeringMethod>(&Problem::steeringMethod),
          (arg("steeringMethod")))
      
      .def("configValidation", 
          static_cast<GetConfigValidation>(&Problem::configValidation),
          return_internal_reference<>())
      .def("configValidation", 
          static_cast<SetConfigValidation>(&Problem::configValidation),
          (arg("configValidation")))
      
      .def("pathValidation", 
          static_cast<GetPathValidation>(&Problem::pathValidation))
      .def("pathValidation", 
          static_cast<SetPathValidation>(&Problem::pathValidation),
          (arg("pathValidation")))
      
      .def("pathProjector", 
          static_cast<GetPathProjector>(&Problem::pathProjector))
      .def("pathProjector", 
          static_cast<SetPathProjector>(&Problem::pathProjector),
          (arg("pathProjector")))
      
      .def("distance", 
          static_cast<GetDistance>(&Problem::distance))
      .def("distance", 
          static_cast<SetDistance>(&Problem::distance),
          (arg("distance")))
      
      .def("target", 
          static_cast<GetTarget>(&Problem::target),
          return_internal_reference<>())
      .def("target", 
          static_cast<SetTarget>(&Problem::target),
          (arg("target")))
      .def("configurationShooter", 
          static_cast<GetConfigurationShooter>(&Problem::configurationShooter))
      .def("configurationShooter", 
          static_cast<SetConfigurationShooter>(&Problem::configurationShooter),
     (arg("configurationShooter")))
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
