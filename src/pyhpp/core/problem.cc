// problem.cpp
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

#include "pyhpp/core/problem.hh"

#include <boost/python.hpp>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <pyhpp/core/steering-method.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/com-between-feet.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/pinocchio/frame.hh>

namespace pyhpp {
namespace core {

using namespace boost::python;

Problem::Problem(const DevicePtr_t& robot)
    : obj(hpp::core::Problem::create(robot)) {}

const DevicePtr_t& Problem::robot() const { return obj->robot(); }

void Problem::setParameter(const std::string& name, const Parameter& value) {
  obj->setParameter(name, value);
}

void Problem::setParameterFloat(const std::string& name, value_type value) {
  Parameter param = Parameter(value);
  obj->setParameter(name, param);
}

void Problem::setParameterInt(const std::string& name, size_type value) {
  Parameter param = Parameter(value);
  obj->setParameter(name, param);
}

void Problem::addConfigValidation(const std::string& type) {
  hpp::core::ConfigValidationPtr_t validation;
  if (type == "CollisionValidation")
    validation = hpp::core::CollisionValidation::create(robot());
  else if (type == "JointBoundValidation")
    validation = hpp::core::JointBoundValidation::create(robot());
  obj->addConfigValidation(validation);
}

void Problem::clearConfigValidations() { obj->clearConfigValidations(); }

const Parameter& Problem::getParameter(const std::string& name) const {
  return obj->getParameter(name);
}

ConfigurationShooterPtr_t Problem::configurationShooter() const {
  return obj->configurationShooter();
}

PyWSteeringMethodPtr_t Problem::steeringMethod() const {
  auto wrapper =
      std::make_shared<pyhpp::core::SteeringMethod>(obj->steeringMethod());
  return wrapper;
}
const ConfigValidationsPtr_t& Problem::configValidation() const {
  return obj->configValidations();
}

PathValidationPtr_t Problem::pathValidation() const {
  return obj->pathValidation();
}

PathProjectorPtr_t Problem::pathProjector() const {
  return obj->pathProjector();
}

DistancePtr_t Problem::distance() const { return obj->distance(); }

const ProblemTargetPtr_t& Problem::target() const { return obj->target(); }

void Problem::configurationShooter(const ConfigurationShooterPtr_t& cs) {
  obj->configurationShooter(cs);
}

void Problem::steeringMethod(const PyWSteeringMethodPtr_t& steeringMethod) {
  obj->steeringMethod(steeringMethod->obj);
}

void Problem::configValidation(const ConfigValidationsPtr_t& cv) {
  obj->configValidation(cv);
}

void Problem::pathValidation(const PathValidationPtr_t& pv) {
  obj->pathValidation(pv);
}

void Problem::pathProjector(const PathProjectorPtr_t& pp) {
  obj->pathProjector(pp);
}

void Problem::distance(const DistancePtr_t& d) { obj->distance(d); }

void Problem::target(const ProblemTargetPtr_t& t) { obj->target(t); }

void Problem::initConfig(ConfigurationIn_t inConfig) {
  obj->initConfig(inConfig);
}

void Problem::addGoalConfig(ConfigurationIn_t config) {
  obj->addGoalConfig(config);
}

void Problem::resetGoalConfigs() { obj->resetGoalConfigs(); }

void Problem::addPartialCom(const std::string& name, boost::python::list pyjointNames) {
  try {
    hpp::pinocchio::CenterOfMassComputationPtr_t comc =
        hpp::pinocchio::CenterOfMassComputation::create(robot());
    auto jointNames = extract_vector<std::string>(pyjointNames);

    for (long unsigned int i = 0; i < jointNames.size(); ++i) {
      std::string name(jointNames[i]);
      hpp::pinocchio::JointPtr_t j = robot()->getJointByName(name);
      if (!j) throw std::logic_error("One joint not found.");
      comc->add(j);
    }
    centerOfMassComputations[name] = comc;
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

hpp::pinocchio::vector3_t Problem::getPartialCom(const std::string& name) {
  try {
    if (!centerOfMassComputations[name]) {
      throw std::logic_error("Partial COM " + name + " not found.");
    }
    hpp::pinocchio::CenterOfMassComputationPtr_t comc =
        centerOfMassComputations[name];
    comc->compute(hpp::pinocchio::COM);
    return comc->com();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

typedef PyWSteeringMethodPtr_t (Problem::*GetSteeringMethod)() const;
typedef void (Problem::*SetSteeringMethod)(const PyWSteeringMethodPtr_t&);

typedef const ConfigValidationsPtr_t& (Problem::*GetConfigValidation)() const;
typedef void (Problem::*SetConfigValidation)(const ConfigValidationsPtr_t&);

typedef PathValidationPtr_t (Problem::*GetPathValidation)() const;
typedef void (Problem::*SetPathValidation)(const PathValidationPtr_t&);

typedef PathProjectorPtr_t (Problem::*GetPathProjector)() const;
typedef void (Problem::*SetPathProjector)(const PathProjectorPtr_t&);

typedef DistancePtr_t (Problem::*GetDistance)() const;
typedef void (Problem::*SetDistance)(const DistancePtr_t&);

typedef const ProblemTargetPtr_t& (Problem::*GetTarget)() const;
typedef void (Problem::*SetTarget)(const ProblemTargetPtr_t&);

typedef ConfigurationShooterPtr_t (Problem::*GetConfigurationShooter)() const;
typedef void (Problem::*SetConfigurationShooter)(
    const ConfigurationShooterPtr_t&);

struct ProblemWrapperConverter {
  static void* convertible(PyObject* obj_ptr) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Problem> extractor(obj);
    return extractor.check() ? obj_ptr : nullptr;
  }

  // Converter for std::shared_ptr<hpp::core::Problem>
  static void construct_shared_ptr(
      PyObject* obj_ptr,
      boost::python::converter::rvalue_from_python_stage1_data* data) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Problem> extractor(obj);

    const Problem& wrapper = extractor();
    void* storage =
        ((boost::python::converter::rvalue_from_python_storage<ProblemPtr_t>*)
             data)
            ->storage.bytes;
    new (storage) ProblemPtr_t(wrapper.obj);
    data->convertible = storage;
  }

  // Converter for std::shared_ptr<hpp::core::Problem const>
  static void construct_const_shared_ptr(
      PyObject* obj_ptr,
      boost::python::converter::rvalue_from_python_stage1_data* data) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Problem> extractor(obj);

    const Problem& wrapper = extractor();
    typedef std::shared_ptr<hpp::core::Problem const> ConstProblemPtr_t;
    void* storage = ((boost::python::converter::rvalue_from_python_storage<
                         ConstProblemPtr_t>*)data)
                        ->storage.bytes;
    new (storage) ConstProblemPtr_t(wrapper.obj);
    data->convertible = storage;
  }
};

void register_problem_converters() {
  // Register converter for std::shared_ptr<hpp::core::Problem>
  boost::python::converter::registry::push_back(
      &ProblemWrapperConverter::convertible,
      &ProblemWrapperConverter::construct_shared_ptr,
      boost::python::type_id<ProblemPtr_t>());

  // Register converter for std::shared_ptr<hpp::core::Problem const>
  boost::python::converter::registry::push_back(
      &ProblemWrapperConverter::convertible,
      &ProblemWrapperConverter::construct_const_shared_ptr,
      boost::python::type_id<std::shared_ptr<hpp::core::Problem const>>());
}

// Python bindings
void exposeProblem() {
  class_<hpp::core::Problem, boost::noncopyable>("CppCoreProblem", no_init);

  class_<Problem>("Problem", init<const DevicePtr_t&>())
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, robot)
      .PYHPP_DEFINE_METHOD(Problem, setParameter)
      .def("setParameter", &Problem::setParameterFloat)
      .def("setParameter", &Problem::setParameterInt)
      .PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(Problem, getParameter)
      .def("steeringMethod",
           static_cast<PyWSteeringMethodPtr_t (Problem::*)() const>(
               &Problem::steeringMethod))
      .def("steeringMethod",
           static_cast<void (Problem::*)(const PyWSteeringMethodPtr_t&)>(
               &Problem::steeringMethod))

      .def("configValidation",
           static_cast<GetConfigValidation>(&Problem::configValidation),
           return_value_policy<copy_const_reference>())
      .def("configValidation",
           static_cast<SetConfigValidation>(&Problem::configValidation),
           (arg("configValidation")))

      .def("addConfigValidation", &Problem::addConfigValidation)
      .def("clearConfigValidations", &Problem::clearConfigValidations)

      .def("pathValidation",
           static_cast<GetPathValidation>(&Problem::pathValidation))
      .def("pathValidation",
           static_cast<SetPathValidation>(&Problem::pathValidation),
           (arg("pathValidation")))

      .def("pathProjector",
           static_cast<GetPathProjector>(&Problem::pathProjector))
      .def("pathProjector",
           static_cast<SetPathProjector>(&Problem::pathProjector),
           (arg("pathProjector")))

      .def("distance", static_cast<GetDistance>(&Problem::distance))
      .def("distance", static_cast<SetDistance>(&Problem::distance),
           (arg("distance")))

      .def("target", static_cast<GetTarget>(&Problem::target),
           return_value_policy<copy_const_reference>())
      .def("target", static_cast<SetTarget>(&Problem::target), (arg("target")))
      .def("configurationShooter",
           static_cast<GetConfigurationShooter>(&Problem::configurationShooter))
      .def("configurationShooter",
           static_cast<SetConfigurationShooter>(&Problem::configurationShooter),
           (arg("configurationShooter")))
      .PYHPP_DEFINE_METHOD(Problem, initConfig)
      .PYHPP_DEFINE_METHOD(Problem, addGoalConfig)
      .PYHPP_DEFINE_METHOD(Problem, resetGoalConfigs)
      .PYHPP_DEFINE_METHOD(Problem, addPartialCom)
      .PYHPP_DEFINE_METHOD(Problem, getPartialCom)
      ;

  register_problem_converters();

  class_<ConstraintResult>("ConstraintResult")
    .def_readwrite("success", &ConstraintResult::success)
    .def_readwrite("configuration", &ConstraintResult::configuration)
    .def_readwrite("error", &ConstraintResult::error);
}

}  // namespace core
}  // namespace pyhpp
