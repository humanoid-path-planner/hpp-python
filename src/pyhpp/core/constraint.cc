//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/constraint.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct CWrapper {
  static bool apply(Constraint& cs, vectorOut_t q) {
    vector_t _q(q);
    bool res = cs.apply(_q);
    q = _q;
    return res;
  }
  static bool isSatisfied1(Constraint& cs, const vector_t& q) {
    return cs.isSatisfied(q);
  }
  static bool isSatisfied2(Constraint& cs, const vector_t& q,
                           vectorOut_t error) {
    vector_t _err(error);
    bool res = cs.isSatisfied(q, _err);
    error = _err;
    return res;
  }
  static ConstraintPtr_t copy(const Constraint* cs) { return cs->copy(); }
};

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
    .def("solver", static_cast<BySubstitution& (ConfigProjector::*)()>(&ConfigProjector::solver), return_internal_reference<>())
          .def("add", &ConfigProjector::add)
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
               return_value_policy<return_by_value>());
}
}  // namespace core
}  // namespace pyhpp
