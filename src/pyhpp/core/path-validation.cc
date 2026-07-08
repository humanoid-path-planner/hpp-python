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
#include <hpp/core/collision-validation.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/core/problem.hh>
#include <pyhpp/util.hh>
// DocNamespace(hpp::core)

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct PVWrapper {
  static bool validate(PathValidation* pv, const PathPtr_t path, bool reverse,
                       PathPtr_t& validPart,
                       PathValidationReportPtr_t& report) {
    return pv->validate(path, reverse, validPart, report);
  }

  static tuple py_validate(PathValidation* pv, const PathPtr_t path,
                           bool reverse = false) {
    PathPtr_t validPart;
    PathValidationReportPtr_t report;
    bool res = pv->validate(path, reverse, validPart, report);
    return boost::python::make_tuple(res, validPart, report);
  }
  static tuple validateConfiguration(PathValidation* pv, ConfigurationIn_t q) {
    ValidationReportPtr_t report;
    bool res = pv->validate(q, report);
    return boost::python::make_tuple(res, report);
  }

  static pathValidation::DiscretizedPtr_t
  createDiscretizedJointBoundAndCollisionChecking(const DevicePtr_t& robot,
                                                  const value_type& stepSize) {
    using namespace pathValidation;
    return Discretized::create(stepSize,
                               {
                                   JointBoundValidation::create(robot),
                                   CollisionValidation::create(robot),
                               });
  }
};
void exposePathValidation() {
  // DocClass(PathValidation)
  class_<PathValidation, PathValidationPtr_t, boost::noncopyable>(
      "PathValidation", DocClassDoc(), no_init)
      .def("validate", &PVWrapper::validate, DocClassMethod(validate))
      .def("validate", &PVWrapper::py_validate,
           "Validate path; returns (valid, validPart, report).")
      .def("validateConfiguration", &PVWrapper::validateConfiguration,
           "Validate a configuration; returns (valid, report).");

  class_<pathValidation::Discretized, bases<PathValidation>,
         hpp::core::pathValidation::DiscretizedPtr_t, boost::noncopyable>(
      "Discretized", DocClassDoc(), no_init)
      .def("__init__",
           make_constructor(
               +[](const DevicePtr_t& robot, const value_type& stepSize) {
                 return pathValidation::createDiscretizedCollisionChecking(
                     robot, stepSize);
               },
               default_call_policies(), (arg("robot"), arg("stepSize"))),
           "Create a discretized collision-checking path validation.");

  hpp::core::continuousValidation::ProgressivePtr_t (*ProgressiveConstructor)(
      const DevicePtr_t&, const value_type&) =
      &continuousValidation::Progressive::create;
  class_<continuousValidation::Progressive, bases<PathValidation>,
         hpp::core::continuousValidation::ProgressivePtr_t, boost::noncopyable>(
      "Progressive", DocClassDoc(), no_init)
      .def("__init__",
           make_constructor(ProgressiveConstructor, default_call_policies(),
                            (arg("robot"), arg("tolerance"))),
           "Create a progressive continuous path validation.");

  hpp::core::continuousValidation::DichotomyPtr_t (*DichotomyConstructor)(
      const DevicePtr_t&, const value_type&) =
      &continuousValidation::Dichotomy::create;
  class_<continuousValidation::Dichotomy, bases<PathValidation>,
         hpp::core::continuousValidation::DichotomyPtr_t, boost::noncopyable>(
      "Dichotomy", DocClassDoc(), no_init)
      .def("__init__",
           make_constructor(DichotomyConstructor, default_call_policies(),
                            (arg("robot"), arg("tolerance"))),
           "Create a dichotomy-based continuous path validation.");

  def("DiscretizedCollision",
      &pathValidation::createDiscretizedCollisionChecking,
      (arg("robot"), arg("stepSize")),
      "Create a discretized collision-checking path validation.");
  def("DiscretizedJointBound", &pathValidation::createDiscretizedJointBound,
      (arg("robot"), arg("stepSize")),
      "Create a discretized joint-bound path validation.");
  def("DiscretizedCollisionAndJointBound",
      &PVWrapper::createDiscretizedJointBoundAndCollisionChecking,
      (arg("robot"), arg("stepSize")),
      "Create a discretized path validation checking both collision and joint "
      "bounds.");
}
}  // namespace core
}  // namespace pyhpp
