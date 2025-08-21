// problem.hh
//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
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

#ifndef PYHPP_MANIPULATION_PROBLEM_HH
#define PYHPP_MANIPULATION_PROBLEM_HH

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/manipulation/problem.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/core/problem.hh>
#include <pyhpp/manipulation/fwd.hh>

typedef hpp::manipulation::ProblemPtr_t ProblemPtr_t;
typedef hpp::constraints::JointAndShape_t JointAndShape_t;
typedef hpp::constraints::JointAndShapes_t JointAndShapes_t;
typedef hpp::core::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
typedef hpp::core::SteeringMethodPtr_t SteeringMethodPtr_t;
typedef hpp::core::ConfigValidationsPtr_t ConfigValidationsPtr_t;
typedef hpp::core::PathValidationPtr_t PathValidationPtr_t;
typedef hpp::core::PathProjectorPtr_t PathProjectorPtr_t;
typedef hpp::core::ProblemTargetPtr_t ProblemTargetPtr_t;
typedef hpp::core::DistancePtr_t DistancePtr_t;

namespace pyhpp {
namespace manipulation {

// Wrapper class for hpp::manipulation::Problem
struct Problem : public pyhpp::core::Problem {
  hpp::core::Container<JointAndShapes_t> jointAndShapes;

  Problem(const PyWDevicePtr_t& robot);
  Problem(const hpp::manipulation::ProblemPtr_t& object);

  void constraintGraph(const PyWGraphPtr_t& graph);
  PyWGraphPtr_t constraintGraph() const;
  virtual void checkProblem() const;
  void initConfig(ConfigurationIn_t inConfig);
  void addGoalConfig(ConfigurationIn_t config);

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

  // PathValidationPtr_t pathValidation() const;
  // void pathValidation (const PathValidationPtr_t &pathValidation);
  // SteeringMethodPtr_t manipulationSteeringMethod() const;
  // PathValidationPtr_t pathValidationFactory() const;
  // void 	setPathValidationFactory (const core::PathValidationBuilder_t
  // &factory, const value_type &tol);
};

}  // namespace manipulation
}  // namespace pyhpp

#endif  // PYHPP_MANIPULATION_PROBLEM_HH
