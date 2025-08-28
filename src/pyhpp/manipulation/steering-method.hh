// problem.hh
//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
//
// This file is part of hpp-python
// hpp-python is free software—you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python. If not, see <http://www.gnu.org/licenses/>.

#ifndef PYHPP_STEERING_METHOD_HH
#define PYHPP_STEERING_METHOD_HH

#include <pyhpp/core/problem.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/manipulation/steering-method/graph.hh>

namespace pyhpp {
namespace manipulation {

using namespace boost::python;
typedef pyhpp::core::PyWSteeringMethodPtr_t PyWSteeringMethodPtr_t;


struct GraphSteeringMethod {
  hpp::manipulation::steeringMethod::GraphPtr_t obj;

  GraphSteeringMethod(const PyWSteeringMethodPtr_t& steeringMethodWrapper);
};


}  // namespace manipulation
}  // namespace pyhpp

#endif  // PYHPP_STEERING_METHOD_HH
