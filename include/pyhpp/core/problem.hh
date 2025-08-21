// problem.hh
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

#ifndef PYHPP_CORE_PROBLEM_HH
#define PYHPP_CORE_PROBLEM_HH

#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/pointer.hh>
#include <pyhpp/core/fwd.hh>

namespace pyhpp {
namespace core {

typedef hpp::core::ProblemPtr_t ProblemPtr_t;
typedef hpp::core::ProblemConstPtr_t ProblemConstPtr_t;
typedef hpp::core::DevicePtr_t DevicePtr_t;
typedef hpp::core::Parameter Parameter;
typedef hpp::core::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
typedef hpp::core::SteeringMethodPtr_t SteeringMethodPtr_t;
typedef hpp::core::ConfigValidationsPtr_t ConfigValidationsPtr_t;
typedef hpp::core::PathValidationPtr_t PathValidationPtr_t;
typedef hpp::core::PathProjectorPtr_t PathProjectorPtr_t;
typedef hpp::core::ProblemTargetPtr_t ProblemTargetPtr_t;
typedef hpp::core::DistancePtr_t DistancePtr_t;

// Wrapper class for hpp::core::Problem
struct Problem {
  hpp::core::ProblemPtr_t obj;

  Problem(const DevicePtr_t& robot);

  // wrapped methods
  const DevicePtr_t& robot() const;
  void setParameter(const std::string& name, const Parameter& value);
  const Parameter& getParameter(const std::string& name) const;
  SteeringMethodPtr_t steeringMethod() const;
  const ConfigValidationsPtr_t& configValidation() const;
  PathValidationPtr_t pathValidation() const;
  PathProjectorPtr_t pathProjector() const;
  DistancePtr_t distance() const;
  const ProblemTargetPtr_t& target() const;
  ConfigurationShooterPtr_t configurationShooter() const;
  void steeringMethod(const SteeringMethodPtr_t& sm);
  void configValidation(const ConfigValidationsPtr_t& cv);
  void pathValidation(const PathValidationPtr_t& pv);
  void pathProjector(const PathProjectorPtr_t& pp);
  void distance(const DistancePtr_t& d);
  void target(const ProblemTargetPtr_t& t);
  void configurationShooter(const ConfigurationShooterPtr_t& configurationShooter);
  void initConfig(ConfigurationIn_t inConfig);
  void addGoalConfig(ConfigurationIn_t config);
};

}  // namespace core
}  // namespace pyhpp

#endif  // PYHPP_CORE_PROBLEM_HH
