// problem.cpp
//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
//
// This file is part of hpp-python
// hpp-python is free software—you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python. If not, see <http://www.gnu.org/licenses/>.

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
