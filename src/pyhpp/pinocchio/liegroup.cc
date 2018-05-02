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

#include <pyhpp/pinocchio/urdf/fwd.hh>

#include <boost/python.hpp>

#include <eigenpy/eigenpy.hpp>

#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/liegroup-element.hh>

using namespace boost::python;

namespace pyhpp {
  namespace pinocchio {
    using namespace hpp::pinocchio;

    void exposeLiegroup()
    {
      class_<LiegroupSpace, LiegroupSpacePtr_t, boost::noncopyable> ("LiegroupSpace", no_init)
        .def ("name", &LiegroupSpace::name, return_value_policy<return_by_value>())
        ;
      class_<LiegroupElement> ("LiegroupElement", init<const vector_t&, const LiegroupSpacePtr_t&>())
        .def ("vector", static_cast <const vector_t& (LiegroupElement::*) () const> (&LiegroupElement::vector), return_value_policy<return_by_value>())
        .def ("space", &LiegroupElement::space, return_value_policy<return_by_value>())
        ;
    }
  }
}
