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

#include <pyhpp/core/fwd.hh>

#include <boost/python.hpp>

#include <hpp/core/path.hh>

#include <pyhpp/util.hh>
#include <hpp/python/config.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    using namespace hpp::core;

    struct HPP_PYTHON_LOCAL PWrapper {
    };

    void exposePath()
    {
      class_<Path, PathPtr_t, boost::noncopyable> ("Path", no_init)
        .def ("__str__", &to_str_from_operator<Path>)

        // .def ("__call__", static_cast<Configuration_t (Path::*) (const value_type&)>(&Path::operator()))
        // TODO check that the boolean is returned to Python
        .def ("__call__", static_cast<Configuration_t (Path::*) (const value_type&, bool&) const>(&Path::operator()))
        ;
    }
  }
}
