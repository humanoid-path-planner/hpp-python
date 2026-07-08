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

namespace {
const char* DOC_PAR_CREATEBOOL = "Create a Parameter holding a boolean value.";
const char* DOC_PAR_BOOL = "Return the boolean value of this parameter.";
const char* DOC_PAR_INT = "Return the integer value of this parameter.";
const char* DOC_PAR_FLOAT =
    "Return the floating-point value of this parameter.";
const char* DOC_PAR_STRING = "Return the string value of this parameter.";
const char* DOC_PAR_VECTOR = "Return the vector value of this parameter.";
const char* DOC_PAR_MATRIX = "Return the matrix value of this parameter.";
const char* DOC_PAR_VALUE =
    "Return the parameter value as a Python object (bool, int, float, str, "
    "numpy array).";
}  // namespace

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
  class_<Parameter>("Parameter", DocClassDoc(), no_init)
      .def("__init__", &create)
      .def("create_bool", &createBool, DOC_PAR_CREATEBOOL)
      .staticmethod("create_bool")
      .def("boolValue", &Parameter::boolValue, DOC_PAR_BOOL)
      .def("intValue", &Parameter::intValue, DOC_PAR_INT)
      .def("floatValue", &Parameter::floatValue, DOC_PAR_FLOAT)
      .def("stringValue", &Parameter::stringValue, DOC_PAR_STRING)
      .def("vectorValue", &Parameter::vectorValue, DOC_PAR_VECTOR)
      .def("matrixValue", &Parameter::matrixValue, DOC_PAR_MATRIX)
      .def("value", &parameter_as_python_object, DOC_PAR_VALUE);
}
}  // namespace core
}  // namespace pyhpp
