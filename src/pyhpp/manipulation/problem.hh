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

#include <pyhpp/core/fwd.hh>
#include <pyhpp/manipulation/fwd.hh>
#include <hpp/manipulation/problem.hh>
#include <pyhpp/core/problem.hh>
#include <hpp/core/configuration-shooter.hh>

typedef hpp::manipulation::ProblemPtr_t ProblemPtr_t;
typedef hpp::core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;

namespace pyhpp {
namespace manipulation {

// Wrapper class for hpp::manipulation::Problem
struct Problem {
  ProblemPtr_t obj;

  Problem(const PyWDevicePtr_t& robot);
  Problem(const hpp::manipulation::ProblemPtr_t& object);

  void constraintGraph (const PyWGraphPtr_t &graph);
  PyWGraphPtr_t constraintGraph() const;
  virtual void checkProblem() const; 
  ConfigurationShooterPtr_t configurationShooter() const;

  // PathValidationPtr_t pathValidation() const; 
  // void pathValidation (const PathValidationPtr_t &pathValidation); 
  // SteeringMethodPtr_t manipulationSteeringMethod() const;
  // PathValidationPtr_t pathValidationFactory() const;
  // void 	setPathValidationFactory (const core::PathValidationBuilder_t &factory, const value_type &tol);
};

}  // namespace manipulation
}  // namespace pyhpp

#endif  // PYHPP_MANIPULATION_PROBLEM_HH
