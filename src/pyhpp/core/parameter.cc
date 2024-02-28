// Copyright (c) 2024
// Authors: Joseph Mirabel
//
// This file is part of hpp-python.
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
// hpp-python. If not, see <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <hpp/core/parameter.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

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
