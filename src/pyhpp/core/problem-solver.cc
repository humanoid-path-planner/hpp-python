//
// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
//
// This file is part of hpp-python
// hpp-python is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python  If not, see
// <http://www.gnu.org/licenses/>.

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
static tuple pathValidationType(const ProblemSolver& ps) {
  value_type tol;
  std::string type = ps.pathValidationType(tol);
  return boost::python::make_tuple(type, tol);
}
static tuple pathProjectorType(const ProblemSolver& ps) {
  value_type tol;
  std::string type = ps.pathProjectorType(tol);
  return boost::python::make_tuple(type, tol);
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
    c.add(key, T(boost::bind(&Builder<Type>::call2, callable, _1)));
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
           return_value_policy<manage_new_object>(), DocClassMethod(create))
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
      //                      DocClassMethod(goalConfigs))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, addGoalConfig,
                            DocClassMethod(addGoalConfig))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetGoalConfigs,
                            DocClassMethod(resetGoalConfigs))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetProblem,
                            DocClassMethod(resetProblem))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, resetRoadmap,
                            DocClassMethod(resetRoadmap))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, createPathOptimizers,
                            DocClassMethod(createPathOptimizers))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, prepareSolveStepByStep,
                            DocClassMethod(prepareSolveStepByStep))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, executeOneStep,
                            DocClassMethod(executeOneStep))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, finishSolveStepByStep,
                            DocClassMethod(finishSolveStepByStep))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, solve, DocClassMethod(solve))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, optimizePath,
                            DocClassMethod(optimizePath))

      .PYHPP_DEFINE_METHOD2(ProblemSolver, addPath, DocClassMethod(addPath))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, erasePath, DocClassMethod(erasePath))
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(ProblemSolver, paths)
      .PYHPP_DEFINE_METHOD2(ProblemSolver, createRobot,
                            DocClassMethod(createRobot))
      .PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF(ProblemSolver, robot,
                                               const DevicePtr_t&)
      .def("addObstacle",
           static_cast<void (ProblemSolver::*)(
               const hpp::pinocchio::DevicePtr_t&, bool, bool)>(
               &ProblemSolver::addObstacle),
           DocClassMethod(addObstacle))

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
                            DocClassMethod(addConfigValidation))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, configValidationTypes,
                            DocClassMethod(configValidationTypes))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, clearConfigValidations,
                            DocClassMethod(clearConfigValidations))

      .PYHPP_DEFINE_METHOD2(ProblemSolver, addPathOptimizer,
                            DocClassMethod(addPathOptimizer))
      .PYHPP_DEFINE_METHOD2(ProblemSolver, clearPathOptimizers,
                            DocClassMethod(clearPathOptimizers))
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
