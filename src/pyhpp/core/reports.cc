//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
