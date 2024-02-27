//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

#include <boost/python.hpp>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

void exposeProblem() {
  register_ptr_to_python<ProblemConstPtr_t>();
  implicitly_convertible<ProblemPtr_t, ProblemConstPtr_t>();
  class_<Problem, ProblemPtr_t, boost::noncopyable>("Problem", no_init)
      // .PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF (Problem, robot, const
      // DevicePtr_t&)
      .def(
          "robot",
          static_cast<const DevicePtr_t& (Problem::*)() const>(&Problem::robot),
          return_value_policy<return_by_value>())
      .def("setParameter", &Problem::setParameter)
      .def("getParameter", &Problem::getParameter, return_value_policy<return_by_value>())
      .def("configurationShooter",
           static_cast<ConfigurationShooterPtr_t (Problem::*)() const>(
               &Problem::configurationShooter))
      .def("steeringMethod",
           static_cast<SteeringMethodPtr_t (Problem::*)() const>(
               &Problem::steeringMethod))
      .def("configValidation",
           static_cast<const ConfigValidationsPtr_t& (Problem::*)() const>(
               &Problem::configValidations),
           return_value_policy<return_by_value>())
      .def("pathValidation",
           static_cast<PathValidationPtr_t (Problem::*)() const>(
               &Problem::pathValidation))
      .def("pathProjector",
           static_cast<PathProjectorPtr_t (Problem::*)() const>(
               &Problem::pathProjector));
}
}  // namespace core
}  // namespace pyhpp
