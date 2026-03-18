//
// Copyright (c) 2026, CNRS
// Authors: Florent Lamiraux
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

#ifndef PYHPP_MANIPULATION_STEERING_METHOD_CARTESIAN_HH
#define PYHPP_MANIPULATION_STEERING_METHOD_CARTESIAN_HH

#include <hpp/manipulation/steering-method/cartesian.hh>
#include <pyhpp/core/problem.hh>

namespace pyhpp {
namespace manipulation {
namespace steeringMethod {

  typedef hpp::constraints::Configuration_t Configuration_t;
  typedef hpp::constraints::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
  typedef hpp::constraints::interval_t interval_t;
  typedef hpp::constraints::ImplicitPtr_t ImplicitPtr_t;
  typedef hpp::constraints::size_type size_type;
  typedef hpp::constraints::value_type value_type;
  typedef hpp::core::PathPtr_t PathPtr_t;

  class Cartesian {
  public:
    hpp::manipulation::steeringMethod::CartesianPtr_t obj;
    Cartesian(const pyhpp::core::Problem& problem);
    void setMaxIterations(size_type iterations);
    size_type getMaxIterations() const;
    void setErrorThreshold(value_type threshold);
    value_type getErrorThreshold() const;
    void setTrajectoryConstraint(const ImplicitPtr_t& ic);
    ImplicitPtr_t getTrajectoryConstraint();
    void setRightHandSide1(const PathPtr_t& rhs, bool se3Output);
    void setRightHandSide2(const DifferentiableFunctionPtr_t& rhs,
			   const interval_t& timeRange);
    DifferentiableFunctionPtr_t getRightHandSide() const;
    interval_t getTimeRange() const;
    size_type getNDiscreteSteps() const;
    void setNDiscreteSteps(size_type n);
    boost::python::tuple planPath(const Configuration_t& q_init);
  };

  void exposeCartesian();
}  // namespace manipulation
}  // namespace pyhpp
} // namespace steeringMethod

#endif
