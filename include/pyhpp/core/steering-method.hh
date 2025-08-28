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

#ifndef PYHPP_CORE_STEERING_METHOD_HH
#define PYHPP_CORE_STEERING_METHOD_HH

#include <hpp/core/steering-method.hh>
#include <hpp/core/path.hh>
#include <hpp/core/config.hh>
#include <pyhpp/core/problem.hh>

namespace pyhpp {
namespace core {

typedef hpp::core::PathPtr_t PathPtr_t;
typedef hpp::core::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::core::ConstraintSetPtr_t ConstraintSetPtr_t;
typedef hpp::core::ProblemConstPtr_t ProblemConstPtr_t;

struct SteeringMethod {
  hpp::core::SteeringMethodPtr_t obj;

  SteeringMethod(hpp::core::SteeringMethodPtr_t obj) : obj(obj) {}

  // Methods from hpp::core::SteeringMethod
  PathPtr_t operator()(ConfigurationIn_t q1, ConfigurationIn_t q2) const;
  PathPtr_t steer(ConfigurationIn_t q1, ConfigurationIn_t q2) const;
  ProblemConstPtr_t problem() const;
  void constraints(const ConstraintSetPtr_t& constraints);
  const ConstraintSetPtr_t& constraints() const;

}; // struct SteeringMethod

} // namespace core
} // namespace pyhpp

#endif