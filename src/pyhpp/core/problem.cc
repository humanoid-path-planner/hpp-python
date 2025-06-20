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

#include <boost/python.hpp>

#include "pyhpp/core/problem.hh"

namespace pyhpp {
namespace core {

using namespace boost::python;


Problem::Problem(const hpp::core::ProblemPtr_t& object)
{
  obj = object;
}
Problem::Problem(const DevicePtr_t& robot) : obj(hpp::core::Problem::create(robot))
{
}

const DevicePtr_t& Problem::robot() const {
  return obj->robot();
}

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

const ProblemTargetPtr_t& Problem::target() const {
  return obj->target();
}

void Problem::initConfig(ConfigurationIn_t inConfig) {
  obj->initConfig(inConfig);
}

void Problem::addGoalConfig(ConfigurationIn_t config) {
  obj->addGoalConfig(config);
}

// Python bindings
void exposeProblem() {
  class_<Problem>("Problem", init <const DevicePtr_t&> ())
      .def("robot",
           static_cast<const DevicePtr_t& (Problem::*)() const>(&Problem::robot),
           return_value_policy<return_by_value>())
      .def("setParameter", &Problem::setParameter)
      .def("getParameter", &Problem::getParameter,
           return_value_policy<return_by_value>())
      .def("configurationShooter",
           &Problem::configurationShooter)
      .def("steeringMethod",
           &Problem::steeringMethod)
      .def("configValidation",
           static_cast<const ConfigValidationsPtr_t& (Problem::*)() const>(
               &Problem::configValidation),
           return_value_policy<return_by_value>())
      .def("pathValidation", &Problem::pathValidation)
      .def("pathProjector", &Problem::pathProjector)
      .def("target",
           static_cast<const ProblemTargetPtr_t& (Problem::*)() const>(
               &Problem::target),
           return_value_policy<return_by_value>())
      .def("initConfig", &Problem::initConfig)
      .def("addGoalConfig", &Problem::addGoalConfig);
}

} // namespace core
} // namespace pyhpp