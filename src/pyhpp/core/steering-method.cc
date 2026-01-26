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

#include <boost/python.hpp>
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method/dubins.hh>
#include <hpp/core/steering-method/hermite.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/steering-method/snibud.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/steering-method/steering-kinodynamic.hh>
#include <hpp/core/steering-method/straight.hh>
#include <pyhpp/core/problem.hh>
#include <pyhpp/core/steering-method.hh>

// DocNamespace(hpp::core)

namespace pyhpp {
namespace core {

using namespace boost::python;

namespace steeringMethod {

// Macro for standard steering methods with create()
#define DEFINE_STEERING_WRAPPER(WrapperName, SteeringType)                  \
  struct WrapperName : public pyhpp::core::SteeringMethod {                 \
    WrapperName(const pyhpp::core::Problem& problem)                        \
        : pyhpp::core::SteeringMethod(SteeringType::create(problem.obj)) {} \
  };                                                                        \
  void expose##WrapperName() {                                              \
    class_<WrapperName, bases<pyhpp::core::SteeringMethod>>(                \
        #WrapperName, init<const pyhpp::core::Problem&>());                 \
  }

// Macro for steering methods with createWithGuess()
#define DEFINE_STEERING_GUESS_WRAPPER(WrapperName, SteeringType) \
  struct WrapperName : public pyhpp::core::SteeringMethod {      \
    WrapperName(const pyhpp::core::Problem& problem)             \
        : pyhpp::core::SteeringMethod(                           \
              SteeringType::createWithGuess(problem.obj)) {}     \
  };                                                             \
  void expose##WrapperName() {                                   \
    class_<WrapperName, bases<pyhpp::core::SteeringMethod>>(     \
        #WrapperName, init<const pyhpp::core::Problem&>());      \
  }

// Use macros for standard create() methods
DEFINE_STEERING_WRAPPER(Straight, hpp::core::steeringMethod::Straight)
DEFINE_STEERING_WRAPPER(Kinodynamic, hpp::core::steeringMethod::Kinodynamic)
DEFINE_STEERING_WRAPPER(Hermite, hpp::core::steeringMethod::Hermite)

// Use macros for createWithGuess() methods
DEFINE_STEERING_GUESS_WRAPPER(ReedsShepp, hpp::core::steeringMethod::ReedsShepp)
DEFINE_STEERING_GUESS_WRAPPER(Dubins, hpp::core::steeringMethod::Dubins)
DEFINE_STEERING_GUESS_WRAPPER(Snibud, hpp::core::steeringMethod::Snibud)

// Special case for templated Spline classes
struct SplineBezier3 : public pyhpp::core::SteeringMethod {
  SplineBezier3(const pyhpp::core::Problem& problem)
      : pyhpp::core::SteeringMethod(
            hpp::core::steeringMethod::Spline<hpp::core::path::BernsteinBasis,
                                              3>::create(problem.obj)) {}
};

void exposeSplineBezier3() {
  class_<SplineBezier3, bases<pyhpp::core::SteeringMethod>>(
      "SplineBezier3", init<const pyhpp::core::Problem&>());
}

struct SplineBezier5 : public pyhpp::core::SteeringMethod {
  SplineBezier5(const pyhpp::core::Problem& problem)
      : pyhpp::core::SteeringMethod(
            hpp::core::steeringMethod::Spline<hpp::core::path::BernsteinBasis,
                                              5>::create(problem.obj)) {}
};

void exposeSplineBezier5() {
  class_<SplineBezier5, bases<pyhpp::core::SteeringMethod>>(
      "SplineBezier5", init<const pyhpp::core::Problem&>());
}

void exposeSteeringMethods() {
  pyhpp::core::steeringMethod::exposeStraight();
  pyhpp::core::steeringMethod::exposeReedsShepp();
  pyhpp::core::steeringMethod::exposeKinodynamic();
  pyhpp::core::steeringMethod::exposeDubins();
  pyhpp::core::steeringMethod::exposeSnibud();
  pyhpp::core::steeringMethod::exposeHermite();
  pyhpp::core::steeringMethod::exposeSplineBezier3();
  pyhpp::core::steeringMethod::exposeSplineBezier5();
}

}  // namespace steeringMethod

// Wrapper methods
PathPtr_t SteeringMethod::operator()(ConfigurationIn_t q1,
                                     ConfigurationIn_t q2) const {
  return (*obj)(q1, q2);
}

PathPtr_t SteeringMethod::steer(ConfigurationIn_t q1,
                                ConfigurationIn_t q2) const {
  return obj->steer(q1, q2);
}

ProblemConstPtr_t SteeringMethod::problem() const { return obj->problem(); }

void SteeringMethod::constraints(const ConstraintSetPtr_t& constraints) {
  obj->constraints(constraints);
}

const ConstraintSetPtr_t& SteeringMethod::constraints() const {
  return obj->constraints();
}

void exposeSteeringMethod() {
  register_ptr_to_python<std::shared_ptr<pyhpp::core::SteeringMethod>>();
  // DocClass(SteeringMethod)
  class_<SteeringMethod>("SteeringMethod", no_init)
      .def("__call__", &SteeringMethod::operator())
      .def("steer", &SteeringMethod::steer, DocClassMethod(steer))
      .def("problem", &SteeringMethod::problem, DocClassMethod(problem))
      .def("constraints",
           static_cast<void (SteeringMethod::*)(const ConstraintSetPtr_t&)>(
               &SteeringMethod::constraints),
           DocClassMethod(constraints))
      .def("constraints",
           static_cast<const ConstraintSetPtr_t& (SteeringMethod::*)() const>(
               &SteeringMethod::constraints),
           return_value_policy<copy_const_reference>());

  pyhpp::core::steeringMethod::exposeSteeringMethods();
}

}  // namespace core
}  // namespace pyhpp
