//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel
//
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef PYHPP_FWD_HH
#define PYHPP_FWD_HH

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/util/pointer.hh>
#include <vector>

#define INIT_PYHPP_MODULE \
  boost::python::docstring_options local_docstring_options(true, true, false)

#define PYHPP_DEFINE_METHOD(CLASS, METHOD) def(#METHOD, &CLASS::METHOD)
#define PYHPP_DEFINE_METHOD1(CLASS, METHOD, ARG1) \
  def(#METHOD, &CLASS::METHOD, ARG1)
#define PYHPP_DEFINE_METHOD2(CLASS, METHOD, ARG1, ARG2) \
  def(#METHOD, &CLASS::METHOD, ARG1, ARG2)
#define PYHPP_DEFINE_METHOD_INTERNAL_REF(CLASS, METHOD) \
  def(#METHOD, &CLASS::METHOD, return_internal_reference<>())
#define PYHPP_DEFINE_METHOD_CONST_REF(CLASS, METHOD) \
  def(#METHOD, &CLASS::METHOD, return_value_policy<copy_const_reference>())
#define PYHPP_DEFINE_METHOD_CONST_REF_BY_VALUE(CLASS, METHOD) \
  def(#METHOD, &CLASS::METHOD, return_value_policy<return_by_value>())
#define PYHPP_DEFINE_GETTER_SETTER(CLASS, METHOD, TYPE)              \
  def(#METHOD, static_cast<TYPE (CLASS::*)() const>(&CLASS::METHOD)) \
      .def(#METHOD, static_cast<void (CLASS::*)(TYPE)>(&CLASS::METHOD))
#define PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF(CLASS, METHOD, TYPE) \
  def(#METHOD, static_cast<TYPE (CLASS::*)() const>(&CLASS::METHOD), \
      return_internal_reference<>())                                 \
      .def(#METHOD, static_cast<void (CLASS::*)(TYPE)>(&CLASS::METHOD))
#define PYHPP_DEFINE_GETTER_SETTER_CONST_REF(CLASS, METHOD, TYPE)    \
  def(#METHOD, static_cast<TYPE (CLASS::*)() const>(&CLASS::METHOD)) \
      .def(#METHOD, static_cast<void (CLASS::*)(const TYPE&)>(&CLASS::METHOD))
#define PYHPP_DEFINE_METHOD_STATIC(CLASS, METHOD) \
  def(#METHOD, &CLASS::METHOD).staticmethod(#METHOD)

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

template <typename T, typename _Vector = std::vector<hpp::shared_ptr<T> > >
struct VectorOfPtr {
  typedef _Vector Vector;
  static T& get_item(Vector& v, std::size_t i) {
    if (i > v.size()) throw std::invalid_argument("Out of range");
    if (!v[i]) throw std::runtime_error("Null pointer");
    return *v[i];
  }
};
template<typename T>
std::vector<T> extract_vector(boost::python::list py_list) {
    std::vector<T> result;
    result.reserve(boost::python::len(py_list));
    
    for (int i = 0; i < boost::python::len(py_list); ++i) {
        boost::python::extract<T> extractor(py_list[i]);
        if (extractor.check()) {
            result.push_back(extractor());
        } else {
            throw std::runtime_error("Failed to extract element");
        }
    }
    return result;
}
template <typename T>
boost::python::list to_python_list(const std::vector<T>& vec) {
  boost::python::list py_list;
  for (const auto& item : vec) {
    py_list.append(item);
  }
  return py_list;
}

}  // namespace pyhpp

#endif  // PYHPP_FWD_HH
