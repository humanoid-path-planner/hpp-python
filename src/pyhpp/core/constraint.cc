//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <eigenpy/eigenpy.hpp>
#include <hpp/constraints/solver/by-substitution.hh>
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
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Constraint, name)
      .PYHPP_DEFINE_METHOD(CWrapper, apply)
      .def("isSatisfied", &CWrapper::isSatisfied1)
      .def("isSatisfied", &CWrapper::isSatisfied2)
      .PYHPP_DEFINE_METHOD(CWrapper, copy);

  class_<ConstraintSet, ConstraintSetPtr_t, boost::noncopyable,
         bases<Constraint> >("ConstraintSet", no_init)
      .PYHPP_DEFINE_METHOD(ConstraintSet, addConstraint)
      .PYHPP_DEFINE_METHOD(ConstraintSet, configProjector);

  class_<ConfigProjector, ConfigProjectorPtr_t, boost::noncopyable,
         bases<Constraint> >("ConfigProjector", no_init)
      .def("solver",
           static_cast<BySubstitution& (ConfigProjector::*)()>(
               &ConfigProjector::solver),
           return_internal_reference<>())
      .def("add", &ConfigProjector::add)
      .PYHPP_DEFINE_GETTER_SETTER(ConfigProjector, lastIsOptional, bool)
      .PYHPP_DEFINE_GETTER_SETTER(ConfigProjector, maxIterations, size_type)
      .def("errorThreshold",
           static_cast<void (ConfigProjector::*)(const value_type&)>(
               &ConfigProjector::errorThreshold))
      .def("errorThreshold",
           static_cast<value_type (ConfigProjector::*)() const>(
               &ConfigProjector::errorThreshold))
      .PYHPP_DEFINE_METHOD(ConfigProjector, residualError)
      .def("sigma", &ConfigProjector::sigma,
           return_value_policy<return_by_value>());
}
}  // namespace core
}  // namespace pyhpp
