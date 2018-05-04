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

#ifndef PYHPP_FWD_HH
#define PYHPP_FWD_HH

#include <boost/python.hpp>

#define PYHPP_DEFINE_METHOD(CLASS, METHOD) .def (#METHOD, &CLASS::METHOD)

namespace pyhpp {
  template <typename ObjectWithPrintMethod>
  std::string to_str (const ObjectWithPrintMethod& obj)
  {
    std::ostringstream oss;
    obj.print(oss);
    return oss.str();
  }
  template <typename ObjectWithPrintMethod>
  std::string to_str_from_operator (const ObjectWithPrintMethod& obj)
  {
    std::ostringstream oss;
    oss << obj;
    return oss.str();
  }
}

#endif // PYHPP_FWD_HH
