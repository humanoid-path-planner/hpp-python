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

#ifndef PYHPP_STD_PAIR_HH
#define PYHPP_STD_PAIR_HH

#include <boost/python.hpp>

namespace pyhpp {
template <typename T1, typename T2>
struct stl_pair {
  typedef std::pair<T1, T2> pair_type;

  stl_pair(const char* name) {
    using namespace boost::python;
    class_<pair_type>(name, init<>())
        .def(init<const T1&, const T2&>())
        .def_readwrite("first", &pair_type::first)
        .def_readwrite("second", &pair_type::second);
  }
};
}  // namespace pyhpp

#endif  // PYHPP_STD_PAIR_HH
