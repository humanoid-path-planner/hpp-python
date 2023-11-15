// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/constraints/hybrid-solver.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/constraint.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct CWrapper {
  static bool apply(Constraint& cs, eigenpy::Ref<vector_t>& q) {
    vector_t _q(q);
    bool res = cs.apply(_q);
    q = _q;
    return res;
  }
  static bool isSatisfied1(Constraint& cs, const vector_t& q) {
    return cs.isSatisfied(q);
  }
  static bool isSatisfied2(Constraint& cs, const vector_t& q,
                           eigenpy::Ref<vector_t>& error) {
    vector_t _err(error);
    bool res = cs.isSatisfied(q, _err);
    error = _err;
    return res;
  }
  static ConstraintPtr_t copy(const Constraint* cs) { return cs->copy(); }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CP_add_overloads, add, 1, 3)

void exposeConstraint() {
  class_<Constraint, ConstraintPtr_t, boost::noncopyable>("Constraint", no_init)
      .def("__str__", &to_str_from_operator<Constraint>)
          PYHPP_DEFINE_METHOD_INTERNAL_REF(Constraint, name)
              PYHPP_DEFINE_METHOD(CWrapper, apply)
      .def("isSatisfied", &CWrapper::isSatisfied1)
      .def("isSatisfied", &CWrapper::isSatisfied2)
          PYHPP_DEFINE_METHOD(CWrapper, copy);

  class_<ConstraintSet, ConstraintSetPtr_t, boost::noncopyable,
         bases<Constraint> >("ConstraintSet", no_init)
      PYHPP_DEFINE_METHOD(ConstraintSet, addConstraint)
          PYHPP_DEFINE_METHOD(ConstraintSet, configProjector);

  class_<ConfigProjector, ConfigProjectorPtr_t, boost::noncopyable,
         bases<Constraint> >("ConfigProjector", no_init)
      PYHPP_DEFINE_METHOD_INTERNAL_REF(ConfigProjector, solver)

          .def("add",
               static_cast<void (ConfigProjector::*)(const LockedJointPtr_t&)>(
                   &ConfigProjector::add))
          .def("add",
               static_cast<bool (ConfigProjector::*)(
                   const NumericalConstraintPtr_t&, const segments_t&,
                   const std::size_t)>(&ConfigProjector::add),
               CP_add_overloads())

              PYHPP_DEFINE_GETTER_SETTER(ConfigProjector, lastIsOptional, bool)
                  PYHPP_DEFINE_GETTER_SETTER(ConfigProjector, maxIterations,
                                             size_type)
          .def("errorThreshold",
               static_cast<void (ConfigProjector::*)(const value_type&)>(
                   &ConfigProjector::errorThreshold))
          .def("errorThreshold",
               static_cast<value_type (ConfigProjector::*)() const>(
                   &ConfigProjector::errorThreshold))
              PYHPP_DEFINE_METHOD(ConfigProjector, residualError)
          .def("sigma", &ConfigProjector::sigma,
               return_value_policy<return_by_value>())
              PYHPP_DEFINE_METHOD(ConfigProjector, numericalConstraints)
                  PYHPP_DEFINE_METHOD(ConfigProjector, lockedJoints);
}
}  // namespace core
}  // namespace pyhpp
