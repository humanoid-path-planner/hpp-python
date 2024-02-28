//
// Copyright (c) 2018 - 2023, CNRS
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

#include <boost/python.hpp>
#include <hpp/core/steering-method.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct SMWrapper {
  static PathPtr_t operator_call(const SteeringMethod& sm, const vector_t& q1,
                                 const vector_t& q2) {
    return sm(q1, q2);
  }
};

void exposeSteeringMethod() {
  class_<SteeringMethod, SteeringMethodPtr_t, boost::noncopyable>(
      "SteeringMethod", no_init)
      .def("__call__", &SMWrapper::operator_call);
}
}  // namespace core
}  // namespace pyhpp
