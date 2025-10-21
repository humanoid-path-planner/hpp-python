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

#include <hpp/manipulation/problem.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/util/pointer.hh>
#include <pyhpp/core/fwd.hh>

namespace pyhpp {
namespace core {

struct SteeringMethod;
typedef std::shared_ptr<SteeringMethod> PyWSteeringMethodPtr_t;

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
typedef hpp::core::value_type value_type;
typedef hpp::core::size_type size_type;
typedef hpp::core::Configuration_t Configuration_t;


struct ConstraintResult {
  bool success;
  Configuration_t configuration;
  value_type error;

  ConstraintResult() : success(false), configuration(), error(0.0) {}
  ConstraintResult(bool s, const Configuration_t& config, value_type err)
      : success(s), configuration(config), error(err) {}
};


// Wrapper class for hpp::core::Problem
struct Problem {
  hpp::core::ProblemPtr_t obj;
  Problem(const DevicePtr_t& robot);
  Problem(hpp::core::ProblemPtr_t problemPtr) : obj(problemPtr) {}

  // wrapped methods
  const DevicePtr_t& robot() const;
  void setParameter(const std::string& name, const Parameter& value);
  void setParameterFloat(const std::string& name, value_type value);
  void setParameterInt(const std::string& name, size_type value);
  const Parameter& getParameter(const std::string& name) const;
  PyWSteeringMethodPtr_t steeringMethod() const;
  const ConfigValidationsPtr_t& configValidation() const;
  PathValidationPtr_t pathValidation() const;
  PathProjectorPtr_t pathProjector() const;
  DistancePtr_t distance() const;
  const ProblemTargetPtr_t& target() const;
  ConfigurationShooterPtr_t configurationShooter() const;
  void steeringMethod(const PyWSteeringMethodPtr_t& steeringMethod);
  void configValidation(const ConfigValidationsPtr_t& cv);
  void clearConfigValidations();
  void pathValidation(const PathValidationPtr_t& pv);
  void pathProjector(const PathProjectorPtr_t& pp);
  void distance(const DistancePtr_t& d);
  void target(const ProblemTargetPtr_t& t);
  void configurationShooter(
      const ConfigurationShooterPtr_t& configurationShooter);
  void initConfig(ConfigurationIn_t inConfig);
  void addGoalConfig(ConfigurationIn_t config);
  void addConfigValidation(const std::string& type);
  void resetGoalConfigs();

  hpp::manipulation::ProblemPtr_t asManipulationProblem() const {
    auto manipProb = HPP_DYNAMIC_PTR_CAST(hpp::manipulation::Problem, obj);
    if (!manipProb) {
      throw std::runtime_error("Not a manipulation problem");
    }
    return manipProb;
  }

  bool isManipulationProblem() const {
    return bool(HPP_DYNAMIC_PTR_CAST(hpp::manipulation::Problem, obj));
  }

  //Constraints utility functions

  void addPartialCom(const std::string& name, boost::python::list pyjointNames);
  hpp::pinocchio::vector3_t getPartialCom(const std::string& name);
  std::map<std::string, hpp::pinocchio::CenterOfMassComputationPtr_t> centerOfMassComputations;
  hpp::constraints::ImplicitPtr_t createRelativeComConstraint(const char* constraintName,
                                          const char* comName,
                                          const char* jointName,
                                          hpp::pinocchio::vector3_t point,
                                          boost::python::list mask);

  hpp::constraints::ImplicitPtr_t createTransformationConstraint(
    const char* constraintName, const char* joint1Name,
    const char* joint2Name, const hpp::pinocchio::Transform3s& M,
    boost::python::list mask);

  hpp::constraints::ImplicitPtr_t createTransformationConstraint2(
    const char* constraintName, const char* joint1Name,
    const char* joint2Name, const hpp::pinocchio::Transform3s& M1,
    const hpp::pinocchio::Transform3s& M2, const boost::python::list mask);

  void setConstantRightHandSide(hpp::constraints::ImplicitPtr_t constraint,
                                bool constant);

  ConstraintResult applyConstraints(ConfigurationIn_t config);
  boost::python::tuple isConfigValid(ConfigurationIn_t dofArray);
  void addNumericalConstraintsToConfigProjector1(const char* configProjName,
                                      boost::python::list constraints,
                                      boost::python::list priorities);
  void addNumericalConstraintsToConfigProjector2(const char* configProjName,
                                        boost::python::list constraints);
  hpp::constraints::ImplicitPtr_t createComBetweenFeet(
    const char* constraintName, const char* comName, const char* jointLName,
    const char* jointRName, const hpp::pinocchio::vector3_t& pointL, 
    const hpp::pinocchio::vector3_t& pointR, const char* jointRefName, 
    const hpp::pinocchio::vector3_t& pointRef, boost::python::list mask);
  hpp::core::ConstraintSetPtr_t constraints_;
  value_type errorThreshold_;
  size_type maxIterProjection_;
};

}  // namespace core
}  // namespace pyhpp

#endif  // PYHPP_CORE_PROBLEM_HH
