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

#include <boost/python.hpp>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/problem.hh>
#include <pyhpp/core/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

void exposePathOptimizer() {
  class_<PathOptimizer, PathOptimizerPtr_t, boost::noncopyable>("PathOptimizer",
                                                                no_init)
      .def("problem", &PathOptimizer::problem,
           return_value_policy<reference_existing_object>());
}
}  // namespace core
}  // namespace pyhpp
