// Copyright (c) 2024
// Authors: Joseph Mirabel
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

#include <boost/python.hpp>
#include <hpp/core/parameter.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::core)

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

object parameter_as_python_object(Parameter* p) {
  switch (p->type()) {
    case Parameter::BOOL:
      return object(p->boolValue());
    case Parameter::INT:
      return object(p->intValue());
    case Parameter::FLOAT:
      return object(p->floatValue());
    case Parameter::STRING:
      return object(p->stringValue());
    case Parameter::VECTOR:
      return object(p->vectorValue());
    case Parameter::MATRIX:
      return object(p->matrixValue());
    default:
    case Parameter::NONE:
      return object();
  }
}

Parameter create(object param) {
#define RETURN_AS(type, variable)                   \
  {                                                 \
    extract<type> get_as(variable);                 \
    if (get_as.check()) return Parameter(get_as()); \
  }

  RETURN_AS(size_type, param);
  // In Python a boolean is an integer so it is not possible to differentiate.
  // RETURN_AS(bool, param);
  RETURN_AS(value_type, param);
  RETURN_AS(std::string, param);
  RETURN_AS(vector_t, param);
  RETURN_AS(matrix_t, param);

  throw std::invalid_argument("cannot build parameter with the argument");

#undef RETURN_AS
}

Parameter createBool(bool param) { return Parameter(param); }

void exposeParameter() {
  // DocClass(Parameter)
  class_<Parameter>("Parameter", no_init)
      .def("create", &create)
      .staticmethod("create")
      .def("create_bool", &createBool)
      .staticmethod("create_bool")
      .PYHPP_DEFINE_METHOD(Parameter, boolValue)
      .PYHPP_DEFINE_METHOD(Parameter, intValue)
      .PYHPP_DEFINE_METHOD(Parameter, floatValue)
      .PYHPP_DEFINE_METHOD(Parameter, stringValue)
      .PYHPP_DEFINE_METHOD(Parameter, vectorValue)
      .PYHPP_DEFINE_METHOD(Parameter, matrixValue)
      .def("value", &parameter_as_python_object);
}
}  // namespace core
}  // namespace pyhpp
