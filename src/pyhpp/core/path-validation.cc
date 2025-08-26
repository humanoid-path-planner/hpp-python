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
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/core/problem.hh>
#include <pyhpp/util.hh>

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
    class_<PathValidation, PathValidationPtr_t, boost::noncopyable>(
        "PathValidation", no_init)
        .def("validate", &PVWrapper::validate)
        .def("validate", &PVWrapper::py_validate)
        .def("validateConfiguration", &PVWrapper::validateConfiguration);

  class_<pathValidation::Discretized, bases<PathValidation>,
         hpp::core::pathValidation::DiscretizedPtr_t, boost::noncopyable>(
      "Discretized", no_init);

  class_<continuousValidation::Progressive, bases<PathValidation>,
         hpp::core::continuousValidation::ProgressivePtr_t, boost::noncopyable>(
      "Progressive", no_init);

  class_<continuousValidation::Dichotomy, bases<PathValidation>,
         hpp::core::continuousValidation::DichotomyPtr_t, boost::noncopyable>(
      "Dichotomy", no_init);

  def("createDiscretized", &pathValidation::createDiscretizedCollisionChecking,
      (arg("robot"), arg("stepSize")));
  def("createDiscretizedCollision",
      &pathValidation::createDiscretizedCollisionChecking,
      (arg("robot"), arg("stepSize")));
  def("createDiscretizedJointBound",
      &pathValidation::createDiscretizedJointBound,
      (arg("robot"), arg("stepSize")));
  def("createDiscretizedCollisionAndJointBound",
      &PVWrapper::createDiscretizedJointBoundAndCollisionChecking,
      (arg("robot"), arg("stepSize")));
  def("createProgressive", &continuousValidation::Progressive::create,
      (arg("robot"), arg("tolerance")));
  def("createDichotomy", &continuousValidation::Dichotomy::create,
      (arg("robot"), arg("tolerance")));
}
}  // namespace core
}  // namespace pyhpp
