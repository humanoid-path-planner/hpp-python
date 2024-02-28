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

#include <boost/python.hpp>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/validation-report.hh>
#include <pinocchio/multibody/fwd.hpp>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

void exposeReports() {
  class_<ValidationReport, ValidationReportPtr_t, boost::noncopyable>(
      "ValidationReport", no_init)
      .def("__str__", &to_str<ValidationReport>);

  class_<CollisionValidationReport, CollisionValidationReportPtr_t,
         bases<ValidationReport> >("CollisionValidationReport", no_init)
      .def_readonly("object1", &CollisionValidationReport::object1)
      .def_readonly("object2", &CollisionValidationReport::object2)
      .def_readonly("result", &CollisionValidationReport::result);

  class_<JointBoundValidationReport, JointBoundValidationReportPtr_t,
         bases<ValidationReport> >("JointBoundValidationReport", no_init)
      .def_readonly("joint_", &JointBoundValidationReport::joint_)
      .def_readonly("rank_", &JointBoundValidationReport::rank_)
      .def_readonly("lowerBound_", &JointBoundValidationReport::lowerBound_)
      .def_readonly("upperBound_", &JointBoundValidationReport::upperBound_)
      .def_readonly("value_", &JointBoundValidationReport::value_);

  class_<PathValidationReport, PathValidationReportPtr_t,
         bases<ValidationReport> >("PathValidationReport", no_init)
      .def_readwrite("parameter", &PathValidationReport::parameter)
      .def_readwrite("configurationReport",
                     &PathValidationReport::configurationReport);

  class_<CollisionPathValidationReport, CollisionPathValidationReportPtr_t,
         bases<CollisionValidationReport> >("CollisionPathValidationReport",
                                            no_init)
      .def("__str__", &to_str<CollisionPathValidationReport>);
}
}  // namespace core
}  // namespace pyhpp
