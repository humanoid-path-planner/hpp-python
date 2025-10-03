//
// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/problem.hh>
#include <hpp/pinocchio/device.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::core)

using namespace boost::python;

#define PYHPP_PROBLEMSOLVER_SELECT_TYPE(type)                        \
  def(#type "Type",                                                  \
      static_cast<const std::string& (ProblemSolver::*)() const>(    \
          &ProblemSolver::type##Type),                               \
      return_value_policy<return_by_value>())                        \
      .def(#type "Type",                                             \
           static_cast<void (ProblemSolver::*)(const std::string&)>( \
               &ProblemSolver::type##Type))

#define PYHPP_PROBLEMSOLVER_CONTAINER(name) \
  def_readwrite(#name, &ProblemSolver::name)

namespace pyhpp {
namespace core {
using namespace hpp::core;

// static list goalConfigs(ProblemSolver& ps) {
//   const Configurations_t cfgs = ps.goalConfigs();
//   list ret;
//   for (std::size_t i = 0; i < cfgs.size(); ++i) ret.append(*cfgs[i]);
//   return ret;
// }
static PathVectorPtr_t path(ProblemSolver& ps, const std::size_t& i) {
  if (i > ps.paths().size()) throw std::invalid_argument("Out of range");
  return ps.paths()[i];
}

template <typename Type, typename TypePtr_t = hpp::shared_ptr<Type> >
struct Builder {
  Builder(PyObject* _callable) : callable(_callable) { incref(callable); }

  ~Builder() { decref(callable); }

  static TypePtr_t call2(PyObject* c, const hpp::core::ProblemConstPtr_t& p) {
    object obj = call<object>(c, p);
    TypePtr_t ptr = extract<TypePtr_t>(obj);
    return ptr;
  }

  template <typename T>
  static void add_to_container(hpp::core::Container<T>& c,
                               const std::string& key, PyObject* callable) {
    // TODO check if incref is needed
    // incref (callable);
    c.add(key, T(boost::bind(&Builder<Type>::call2, callable,
                             boost::placeholders::_1)));
  }

  PyObject* callable;
};

struct NotABuilder {
  template <typename T>
  static void add_to_container(hpp::core::Container<T>& c,
                               const std::string& key, const T& value) {
    c.add(key, value);
  }
};

/// \tparam Builder_t void means not a Builder.
template <typename T, typename Builder_t = NotABuilder>
struct exposeContainer {
  static void run(const char* name) {
    typedef hpp::core::Container<T> C_t;
    class_<C_t>(name, no_init)
        .PYHPP_DEFINE_METHOD(C_t, erase)
        .PYHPP_DEFINE_METHOD(C_t, clear)
        .def("add", &Builder_t::template add_to_container<T>)
        .PYHPP_DEFINE_METHOD(C_t, has)
        // .PYHPP_DEFINE_METHOD (C_t, get)
        .def(
            "__getitem__",
            static_cast<const T& (C_t::*)(const typename C_t::key_type&) const>(
                &C_t::get),
            return_internal_reference<>())
        // .PYHPP_DEFINE_METHOD (C_t, getKeys)
        .def("keys", &C_t::template getKeys<std::vector<std::string> >);
  }
};

void exposeProblemSolver() {
  // DocClass (ProblemSolver)
  using namespace hpp::core;
  class_<ProblemSolver>("ProblemSolver", no_init)
      .def("create", &ProblemSolver::create,
           return_value_policy<manage_new_object>(), "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::create is not documented\n", "")
      .staticmethod("create")
      .def("problem",
           static_cast<ProblemPtr_t (ProblemSolver::*)()>(
               &ProblemSolver::problem),
           return_value_policy<return_by_value>())
      // .def("initConfig", static_cast<const Configuration_t&
      //      (ProblemSolver::*) () const>(&ProblemSolver::initConfig),
      //      return_internal_reference<>())
      .def("path", path)
      .def("initConfig",
           static_cast<const Configuration_t& (ProblemSolver::*)() const>(
               &ProblemSolver::initConfig),
           return_internal_reference<>())
      .def("initConfig",
           static_cast<void (ProblemSolver::*)(ConfigurationIn_t)>(
               &ProblemSolver::initConfig))
      // .PYHPP_DEFINE_METHOD2(ProblemSolver, goalConfigs,
      //                      "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::goalConfigs is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, addGoalConfig,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::addGoalConfig is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetGoalConfigs,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::resetGoalConfigs is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetProblem,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::resetProblem is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetRoadmap,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::resetRoadmap is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, createPathOptimizers,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::createPathOptimizers is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, prepareSolveStepByStep,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::prepareSolveStepByStep is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, executeOneStep,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::executeOneStep is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, finishSolveStepByStep,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::finishSolveStepByStep is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, solve, "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::solve is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, optimizePath,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::optimizePath is not documented\n", "")

      .PYHPP_DEFINE_METHOD2(ProblemSolver, addPath, "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::addPath is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, erasePath, "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::erasePath is not documented\n", "")
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(ProblemSolver, paths)
      .PYHPP_DEFINE_METHOD2(ProblemSolver, createRobot,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::createRobot is not documented\n", "")
      .PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF(ProblemSolver, robot,
                                               const DevicePtr_t&)
      .def("addObstacle",
           static_cast<void (ProblemSolver::*)(
               const hpp::pinocchio::DevicePtr_t&, bool, bool)>(
               &ProblemSolver::addObstacle),
           "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::addObstacle is not documented\n", "")

      .PYHPP_PROBLEMSOLVER_SELECT_TYPE(robot)
      .PYHPP_PROBLEMSOLVER_SELECT_TYPE(pathPlanner)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(ProblemSolver, pathPlanner)
      .PYHPP_PROBLEMSOLVER_SELECT_TYPE(distance)
      .PYHPP_PROBLEMSOLVER_SELECT_TYPE(steeringMethod)
      .PYHPP_PROBLEMSOLVER_SELECT_TYPE(configurationShooter)
      .def("pathValidationType", static_cast<void (ProblemSolver::*)(
                                     const std::string&, const value_type&)>(
                                     &ProblemSolver::pathValidationType))
      .def("pathProjectorType", static_cast<void (ProblemSolver::*)(
                                    const std::string&, const value_type&)>(
                                    &ProblemSolver::pathProjectorType))

      .PYHPP_DEFINE_METHOD2(ProblemSolver, addConfigValidation,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::addConfigValidation is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, configValidationTypes,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::configValidationTypes is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, clearConfigValidations,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::clearConfigValidations is not documented\n", "")

      .PYHPP_DEFINE_METHOD2(ProblemSolver, addPathOptimizer,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::addPathOptimizer is not documented\n", "")
      .PYHPP_DEFINE_METHOD2(ProblemSolver, clearPathOptimizers,
                            "Cannot find doxygen-xml for package hpp-core. hpp::core::ProblemSolver::clearPathOptimizers is not documented\n", "")
      //.PYHPP_DEFINE_METHOD_INTERNAL_REF(ProblemSolver,
      //                                 pathOptimizer)

      // .PYHPP_PROBLEMSOLVER_CONTAINER(robots)
      .PYHPP_PROBLEMSOLVER_CONTAINER(pathPlanners)
      .PYHPP_PROBLEMSOLVER_CONTAINER(pathOptimizers)

      .PYHPP_DEFINE_GETTER_SETTER(ProblemSolver, maxIterProjection, size_type)
      .PYHPP_DEFINE_GETTER_SETTER(ProblemSolver, maxIterPathPlanning, size_type)
      // .PYHPP_DEFINE_GETTER_SETTER (ProblemSolver, errorThreshold     ,
      // value_type)
      .def("errorThreshold", static_cast<value_type (ProblemSolver::*)() const>(
                                 &ProblemSolver::errorThreshold))
      .def("errorThreshold",
           static_cast<void (ProblemSolver::*)(const value_type&)>(
               &ProblemSolver::errorThreshold));

  // exposeContainer<RobotBuilder_t>();
  exposeContainer<PathPlannerBuilder_t>::run("PathPlannerContainer");
  exposeContainer<PathOptimizerBuilder_t, Builder<PathOptimizer> >::run(
      "PathOptimizerContainer");
}
}  // namespace core
}  // namespace pyhpp
