//
// Copyright (c) 2018 - 2026, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux, Paul Sardin
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
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <hpp/core/problem-solver.hh>
#include <pyhpp/core/path-validation.hh>
#include <pyhpp/util.hh>
// DocNamespace(hpp::core)

namespace pyhpp {
namespace core {

using namespace boost::python;

namespace {

hpp::core::PathValidationPtr_t createDiscretizedJointBoundAndCollisionChecking(
    const hpp::core::DevicePtr_t& robot,
    const hpp::core::value_type& stepSize) {
  return hpp::core::pathValidation::Discretized::create(
      stepSize, {hpp::core::JointBoundValidation::create(robot),
                 hpp::core::CollisionValidation::create(robot)});
}

const hpp::core::PathValidationBuilder_t& noValidationFactory() {
  static const hpp::core::PathValidationBuilder_t factory =
      hpp::core::ProblemSolver::create()->pathValidations.get("NoValidation");
  return factory;
}

struct PVWrapper {
  static bool validate(PathValidation* pv, const hpp::core::PathPtr_t path,
                       bool reverse, hpp::core::PathPtr_t& validPart,
                       hpp::core::PathValidationReportPtr_t& report) {
    return pv->obj->validate(path, reverse, validPart, report);
  }

  static tuple pyValidate(PathValidation* pv, const hpp::core::PathPtr_t path,
                          bool reverse = false) {
    hpp::core::PathPtr_t validPart;
    hpp::core::PathValidationReportPtr_t report;
    bool result = pv->obj->validate(path, reverse, validPart, report);
    return boost::python::make_tuple(result, validPart, report);
  }

  static tuple validateConfiguration(PathValidation* pv,
                                     hpp::core::ConfigurationIn_t q) {
    hpp::core::ValidationReportPtr_t report;
    bool result = pv->obj->validate(q, report);
    return boost::python::make_tuple(result, report);
  }
};
namespace pathValidation {

struct NoValidation : PathValidation {
  NoValidation(const hpp::core::DevicePtr_t& robot,
               const hpp::core::value_type& tolerance)
      : PathValidation(robot, noValidationFactory(), tolerance) {}
};

struct Discretized : PathValidation {
  Discretized(const hpp::core::DevicePtr_t& robot,
              const hpp::core::value_type& stepSize)
      : PathValidation(
            robot,
            hpp::core::pathValidation::createDiscretizedCollisionChecking,
            stepSize) {}
};

struct DiscretizedCollision : PathValidation {
  DiscretizedCollision(const hpp::core::DevicePtr_t& robot,
                       const hpp::core::value_type& stepSize)
      : PathValidation(
            robot,
            hpp::core::pathValidation::createDiscretizedCollisionChecking,
            stepSize) {}
};

struct DiscretizedJointBound : PathValidation {
  DiscretizedJointBound(const hpp::core::DevicePtr_t& robot,
                        const hpp::core::value_type& stepSize)
      : PathValidation(robot,
                       hpp::core::pathValidation::createDiscretizedJointBound,
                       stepSize) {}
};

struct DiscretizedCollisionAndJointBound : PathValidation {
  DiscretizedCollisionAndJointBound(const hpp::core::DevicePtr_t& robot,
                                    const hpp::core::value_type& stepSize)
      : PathValidation(robot, createDiscretizedJointBoundAndCollisionChecking,
                       stepSize) {}
};

struct Progressive : PathValidation {
  Progressive(const hpp::core::DevicePtr_t& robot,
              const hpp::core::value_type& tolerance)
      : PathValidation(robot,
                       hpp::core::continuousValidation::Progressive::create,
                       tolerance) {}
};

struct ProgressiveWrapper {
  static hpp::core::value_type getTimeOut(PathValidation* pv) {
    return HPP_DYNAMIC_PTR_CAST(hpp::core::continuousValidation::Progressive,
                                pv->obj)
        ->timeOut();
  }
  static void setTimeOut(PathValidation* pv,
                         const hpp::core::value_type& timeOut) {
    HPP_DYNAMIC_PTR_CAST(hpp::core::continuousValidation::Progressive, pv->obj)
        ->timeOut(timeOut);
  }
};

struct Dichotomy : PathValidation {
  Dichotomy(const hpp::core::DevicePtr_t& robot,
            const hpp::core::value_type& tolerance)
      : PathValidation(robot,
                       hpp::core::continuousValidation::Dichotomy::create,
                       tolerance) {}
};

}  // namespace pathValidation
}  // namespace

void exposePathValidation() {
  register_ptr_to_python<PyWPathValidationPtr_t>();

  // DocClass(PathValidation)
  class_<PathValidation, PyWPathValidationPtr_t, boost::noncopyable>(
      "PathValidation", DocClassDoc(), no_init)
      .def("validate", &PVWrapper::validate, DocClassMethod(validate))
      .def("validate", &PVWrapper::pyValidate,
           "Validate path; returns (valid, validPart, report).")
      .def("validateConfiguration", &PVWrapper::validateConfiguration,
           "Validate a configuration; returns (valid, report).");

  class_<pathValidation::NoValidation, bases<PathValidation>>(
      "NoValidation", "Create a path validation that accepts every path.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("tolerance"))));
  class_<pathValidation::Discretized, bases<PathValidation>>(
      "Discretized", "Create a discretized collision-checking path validation.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("stepSize"))));
  class_<pathValidation::DiscretizedCollision, bases<PathValidation>>(
      "DiscretizedCollision",
      "Create a discretized collision-checking path validation.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("stepSize"))));
  class_<pathValidation::DiscretizedJointBound, bases<PathValidation>>(
      "DiscretizedJointBound",
      "Create a discretized joint-bound path validation.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("stepSize"))));
  class_<pathValidation::DiscretizedCollisionAndJointBound,
         bases<PathValidation>>(
      "DiscretizedCollisionAndJointBound",
      "Create a discretized path validation checking both collision and joint "
      "bounds.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("stepSize"))));
  class_<pathValidation::Progressive, bases<PathValidation>>(
      "Progressive", "Create a progressive continuous path validation.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("tolerance"))))
      .def("timeOut", &pathValidation::ProgressiveWrapper::getTimeOut,
           "Get wall-clock timeout (seconds) for path validation.")
      .def("timeOut", &pathValidation::ProgressiveWrapper::setTimeOut,
           "Set wall-clock timeout (seconds) for path validation.");
  class_<pathValidation::Dichotomy, bases<PathValidation>>(
      "Dichotomy", "Create a dichotomy-based continuous path validation.",
      init<const hpp::core::DevicePtr_t&, const hpp::core::value_type&>(
          (arg("robot"), arg("tolerance"))));
}

}  // namespace core
}  // namespace pyhpp
