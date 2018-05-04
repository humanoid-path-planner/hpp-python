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

#include <eigenpy/eigenpy.hpp>

#include <hpp/core/path.hh>

#include <pyhpp/util.hh>
#include <hpp/python/config.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    using namespace hpp::core;

    struct HPP_PYTHON_LOCAL PWrapper {
      static tuple py_call1 (const Path& p, const value_type& t)
      {
        bool success;
        Configuration_t q (p(t, success));
        return make_tuple (q, success);
      }
      static bool py_call2 (const Path& p, eigenpy::Ref<Configuration_t> q, const value_type& t)
      {
        Configuration_t qq (q);
        bool success = p (qq, t);
        q = qq;
        return success;
      }
    };

    void exposePath()
    {
      class_<Path, PathPtr_t, boost::noncopyable> ("Path", no_init)
        .def ("__str__", &to_str_from_operator<Path>)

        .def ("__call__", &PWrapper::py_call1)
        .def ("__call__", &PWrapper::py_call2)

        .def ("copy", static_cast<PathPtr_t (Path::*) () const> (&Path::copy))
        PYHPP_DEFINE_METHOD (Path, extract)
        PYHPP_DEFINE_METHOD (Path, reverse)
        // PYHPP_DEFINE_METHOD (Path, timeRange)
        .def ("timeRange", static_cast<const interval_t& (Path::*) () const> (&Path::timeRange), return_internal_reference<>())
        PYHPP_DEFINE_METHOD (Path, length)
        PYHPP_DEFINE_METHOD (Path, initial)
        PYHPP_DEFINE_METHOD (Path, end)
        // PYHPP_DEFINE_METHOD (Path, constraints)
        ;
    }
  }
}
