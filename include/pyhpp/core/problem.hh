// problem.hh
//
// Copyright (c) 2025, CNRS
// Authors: <Your Name>
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

  Problem(const hpp::core::ProblemPtr_t& object);
  Problem(const DevicePtr_t& robot);

  // wrapped methods
  const DevicePtr_t& robot() const;
  void setParameter(const std::string& name, const Parameter& value);
  const Parameter& getParameter(const std::string& name) const;
  ConfigurationShooterPtr_t configurationShooter() const;
  SteeringMethodPtr_t steeringMethod() const;
  const ConfigValidationsPtr_t& configValidation() const;
  PathValidationPtr_t pathValidation() const;
  PathProjectorPtr_t pathProjector() const;
  DistancePtr_t distance() const;
  const ProblemTargetPtr_t& target() const;
  void initConfig(ConfigurationIn_t inConfig);
  void addGoalConfig(ConfigurationIn_t config);
};

}  // namespace core
}  // namespace pyhpp

#endif  // PYHPP_CORE_PROBLEM_HH
