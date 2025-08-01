// problem.cpp
//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
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

#include "pyhpp/core/problem.hh"

#include <boost/python.hpp>

namespace pyhpp {
namespace core {

using namespace boost::python;

Problem::Problem(const hpp::core::ProblemPtr_t& object) { obj = object; }
Problem::Problem(const DevicePtr_t& robot)
    : obj(hpp::core::Problem::create(robot)) {}

const DevicePtr_t& Problem::robot() const { return obj->robot(); }

void Problem::setParameter(const std::string& name, const Parameter& value) {
  obj->setParameter(name, value);
}

const Parameter& Problem::getParameter(const std::string& name) const {
  return obj->getParameter(name);
}

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

void Problem::initConfig(ConfigurationIn_t inConfig) {
  obj->initConfig(inConfig);
}

void Problem::addGoalConfig(ConfigurationIn_t config) {
  obj->addGoalConfig(config);
}

// Python bindings
void exposeProblem() {
  class_<Problem>("Problem", init<const DevicePtr_t&>())
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, robot)
      .PYHPP_DEFINE_METHOD(Problem, setParameter)
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, getParameter)
      .PYHPP_DEFINE_METHOD(Problem, configurationShooter)
      .PYHPP_DEFINE_METHOD(Problem, steeringMethod)
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, configValidation)
      .PYHPP_DEFINE_METHOD(Problem, pathValidation)
      .PYHPP_DEFINE_METHOD(Problem, pathProjector)
      .PYHPP_DEFINE_METHOD(Problem, distance)
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, target)
      .PYHPP_DEFINE_METHOD(Problem, initConfig)
      .PYHPP_DEFINE_METHOD(Problem, addGoalConfig);
}

}  // namespace core
}  // namespace pyhpp
