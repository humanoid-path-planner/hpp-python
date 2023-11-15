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
#include <vector>

#define INIT_PYHPP_MODULE \
  boost::python::docstring_options local_docstring_options(true, true, false)

#define PYHPP_DEFINE_METHOD(CLASS, METHOD) .def(#METHOD, &CLASS::METHOD)
#define PYHPP_DEFINE_METHOD1(CLASS, METHOD, ARG1) \
  .def(#METHOD, &CLASS::METHOD, ARG1)
#define PYHPP_DEFINE_METHOD2(CLASS, METHOD, ARG1, ARG2) \
  .def(#METHOD, &CLASS::METHOD, ARG1, ARG2)
#define PYHPP_DEFINE_METHOD_INTERNAL_REF(CLASS, METHOD) \
  .def(#METHOD, &CLASS::METHOD, return_internal_reference<>())
#define PYHPP_DEFINE_GETTER_SETTER(CLASS, METHOD, TYPE)               \
  .def(#METHOD, static_cast<TYPE (CLASS::*)() const>(&CLASS::METHOD)) \
      .def(#METHOD, static_cast<void (CLASS::*)(TYPE)>(&CLASS::METHOD))
#define PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF(CLASS, METHOD, TYPE)  \
  .def(#METHOD, static_cast<TYPE (CLASS::*)() const>(&CLASS::METHOD), \
       return_internal_reference<>())                                 \
      .def(#METHOD, static_cast<void (CLASS::*)(TYPE)>(&CLASS::METHOD))

namespace pyhpp {
template <typename ObjectWithPrintMethod>
std::string to_str(const ObjectWithPrintMethod& obj) {
  std::ostringstream oss;
  obj.print(oss);
  return oss.str();
}
template <typename ObjectWithPrintMethod>
std::string to_str_from_operator(const ObjectWithPrintMethod& obj) {
  std::ostringstream oss;
  oss << obj;
  return oss.str();
}

template <typename T, typename _Vector = std::vector<boost::shared_ptr<T> > >
struct VectorOfPtr {
  typedef _Vector Vector;
  static T& get_item(Vector& v, std::size_t i) {
    if (i > v.size()) throw std::invalid_argument("Out of range");
    if (!v[i]) throw std::runtime_error("Null pointer");
    return *v[i];
  }
};
}  // namespace pyhpp

#endif  // PYHPP_FWD_HH
