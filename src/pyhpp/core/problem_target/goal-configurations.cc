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
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <pyhpp/core/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
namespace problemTarget {
using namespace hpp::core::problemTarget;

void exposeGoalConfigurations() {
  class_<GoalConfigurations, GoalConfigurationsPtr_t,
         bases<hpp::core::ProblemTarget>, boost::noncopyable>(
      "GoalConfigurations", no_init)
      .def("create", &GoalConfigurations::create)
      .PYHPP_DEFINE_METHOD(GoalConfigurations, computePath)
      .PYHPP_DEFINE_METHOD(GoalConfigurations, reached)

      ;
}
}  // namespace problemTarget
}  // namespace core
}  // namespace pyhpp
