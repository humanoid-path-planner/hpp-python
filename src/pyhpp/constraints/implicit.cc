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

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/explicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>
#include <pyhpp/vector-indexing-suite.hh>

#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

void exposeImplicit()
{
  enum_<ComparisonType>("ComparisonType")
    .value("Equality", Equality)
    .value("EqualToZero", EqualToZero)
    .value("Superior", Superior)
    .value("Inferior", Inferior);
  class_<Implicit, ImplicitPtr_t, boost::noncopyable>("Implicit", no_init)
    .def("create", &Implicit::create).staticmethod("create")
    .PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF(Implicit, comparisonType,
                                             const ComparisonTypes_t&)
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Implicit, function)
    .def("parameterSize", &Implicit::parameterSize)
    .def("rightHandSideSize", &Implicit::rightHandSideSize);
}
}  // namespace constraints
}  // namespace pyhpp
