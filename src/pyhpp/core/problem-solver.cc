//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
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

#include <pyhpp/core/fwd.hh>

#include <boost/python.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/core/problem-solver.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

#define PYHPP_PROBLEMSOLVER_SELECT_TYPE(type) \
  .def (#type "Type", static_cast<const std::string& (ProblemSolver::*) () const>(&ProblemSolver:: type ## Type), return_internal_reference<>()) \
  .def (#type "Type", static_cast<void (ProblemSolver::*) (const std::string& ) >(&ProblemSolver:: type ## Type))

#define PYHPP_PROBLEMSOLVER_CONTAINER(name) \
  .def_readwrite (#name, &ProblemSolver::name)

namespace pyhpp {
  namespace core {
    using namespace hpp::core;

    template <typename T>
    void exposeContainer (const char* name)
    {
      typedef Container<T> C_t;
      class_ <C_t> (name, no_init)
        PYHPP_DEFINE_METHOD (C_t, erase)
        PYHPP_DEFINE_METHOD (C_t, clear)
        PYHPP_DEFINE_METHOD (C_t, add)
        PYHPP_DEFINE_METHOD (C_t, has)
        // PYHPP_DEFINE_METHOD (C_t, get)
        .def ("__getitem__", static_cast<const T& (C_t::*) (const typename C_t::key_type&) const> (&C_t::get), return_internal_reference<>())
        // PYHPP_DEFINE_METHOD (C_t, getKeys)
        .def ("keys", &C_t::template getKeys< std::vector<std::string> >)
        ;
    }

    void exposeProblemSolver()
    {
      class_<ProblemSolver> ("ProblemSolver", no_init)
        .def("create", &ProblemSolver::create, return_value_policy<manage_new_object>())
        .staticmethod("create")
        // PYHPP_DEFINE_METHOD (ProblemSolver, problem)
        // .def ("initConfig", static_cast<const ConfigurationPtr_t (ProblemSolver::*) () const>(&ProblemSolver::initConfig))
        // .def ("initConfig", static_cast<void (ProblemSolver::*) (const ConfigurationPtr_t&) >(&ProblemSolver::initConfig))
        // PYHPP_DEFINE_METHOD (ProblemSolver, goalConfigs)
        PYHPP_DEFINE_METHOD (ProblemSolver, addGoalConfig)
        PYHPP_DEFINE_METHOD (ProblemSolver, resetGoalConfigs)

        PYHPP_DEFINE_METHOD (ProblemSolver, createRobot)

        PYHPP_PROBLEMSOLVER_SELECT_TYPE (robot)
        PYHPP_PROBLEMSOLVER_SELECT_TYPE (pathPlanner)
        PYHPP_PROBLEMSOLVER_SELECT_TYPE (distance)
        PYHPP_PROBLEMSOLVER_SELECT_TYPE (steeringMethod)
        PYHPP_PROBLEMSOLVER_SELECT_TYPE (configurationShooter)
        .def ("pathValidationType", static_cast<const std::string& (ProblemSolver::*) (value_type&) /*const*/   >(&ProblemSolver::pathValidationType), return_internal_reference<>()) \
        .def ("pathValidationType", static_cast<void (ProblemSolver::*) (const std::string&, const value_type&) >(&ProblemSolver::pathValidationType))
        .def ("pathProjectorType", static_cast<const std::string& (ProblemSolver::*) (value_type&) const       >(&ProblemSolver::pathProjectorType), return_internal_reference<>()) \
        .def ("pathProjectorType", static_cast<void (ProblemSolver::*) (const std::string&, const value_type&) >(&ProblemSolver::pathProjectorType))
        PYHPP_DEFINE_METHOD (ProblemSolver, addConfigValidation)
        PYHPP_DEFINE_METHOD (ProblemSolver, configValidationTypes)
        PYHPP_DEFINE_METHOD (ProblemSolver, clearConfigValidations)

        // PYHPP_PROBLEMSOLVER_CONTAINER(robots)
        PYHPP_PROBLEMSOLVER_CONTAINER(pathOptimizers)
        ;

      // exposeContainer<RobotBuilder_t>();
      exposeContainer<PathOptimizerBuilder_t>("PathOptimizerContainer");
    }
  }
}
