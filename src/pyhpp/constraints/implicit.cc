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
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/explicit.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>
#include <pyhpp/vector-indexing-suite.hh>

// DocNamespace(hpp::constraints)

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

static size_type getFunctionOutputSize(const ImplicitPtr_t& constraint) {
  return constraint->function().outputSize();
}

void exposeImplicit() {
  enum_<ComparisonType>("ComparisonType")
      .value("Equality", Equality)
      .value("EqualToZero", EqualToZero)
      .value("Superior", Superior)
      .value("Inferior", Inferior);
  // DocClass(Implicit)
  class_<Implicit, ImplicitPtr_t, boost::noncopyable>("Implicit", no_init)
      .def("__init__", make_constructor(&Implicit::create))
      .def("comparisonType",
           static_cast<const ComparisonTypes_t& (Implicit::*)() const>(
               &Implicit::comparisonType),
           return_internal_reference<>())
      .def("comparisonType",
           static_cast<void (Implicit::*)(const ComparisonTypes_t&)>(
               &Implicit::comparisonType))
      .def("function", &Implicit::function, return_internal_reference<>(),
           DocClassMethod(function))
      .def("parameterSize", &Implicit::parameterSize,
           DocClassMethod(parameterSize))
      .def("rightHandSideSize", &Implicit::rightHandSideSize,
           DocClassMethod(rightHandSideSize))
      .def("getFunctionOutputSize", &getFunctionOutputSize)
      .staticmethod("getFunctionOutputSize");
}
}  // namespace constraints
}  // namespace pyhpp
