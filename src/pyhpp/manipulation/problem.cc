//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
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

#include <hpp/manipulation/problem.hh>

using namespace boost::python;

namespace pyhpp {
namespace manipulation {

typedef hpp::manipulation::Problem Problem;
typedef hpp::manipulation::ProblemPtr_t ProblemPtr_t;
typedef hpp::manipulation::ProblemConstPtr_t ProblemConstPtr_t;

void exposeProblem() {
  register_ptr_to_python<ProblemConstPtr_t>();
  implicitly_convertible<ProblemPtr_t, ProblemConstPtr_t>();
  class_<Problem, ProblemPtr_t, boost::noncopyable, bases <hpp::core::Problem> >
    ("Problem", no_init).
    def("create", &Problem::create).staticmethod("create");
}
} // manipulation
} // pyhpp
