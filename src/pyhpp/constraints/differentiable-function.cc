//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/constraints/differentiable-function.hh>
#include <pyhpp/constraints/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::constraints)

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

struct DFWrapper : DifferentiableFunction, wrapper<DifferentiableFunction> {
  typedef hpp::shared_ptr<DFWrapper> Ptr_t;

  DFWrapper(size_type sizeInput, size_type sizeInputDerivative,
            size_type sizeOutput, std::string name)
      : DifferentiableFunction(sizeInput, sizeInputDerivative, sizeOutput,
                               name) {}

  void impl_compute(LiegroupElementRef result, vectorIn_t argument) const {
    override f = this->get_override("impl_compute");
    if (!f)
      throw std::runtime_error("impl_compute not implemented in child class");
    f(boost::ref(result), vector_t(argument));
  }

  void impl_jacobian(matrixOut_t jacobian, vectorIn_t arg) const {
    override f = this->get_override("impl_jacobian");
    if (!f)
      throw std::runtime_error("impl_compute not implemented in child class");
    matrix_t J(outputDerivativeSize(), inputDerivativeSize());
    jacobian = (f(J, vector_t(arg))).as<matrix_t>();
  }

  // TODO add virtual method print
  // so that it can be reimplemented in Python

  static void value_wrap(const DifferentiableFunction& f, LiegroupElement& lge,
                         const Configuration_t& q) {
    f.value(lge, q);
  }

  // NOTE: eigenpy::Ref<const vector_t> is not binded so far. It is not very
  // useful since it cannot be used to return a value.
  static void jacobian_wrap(const DifferentiableFunction& f, matrixOut_t J,
                            const Configuration_t& q) {
    J = py_jacobian(f, q);
    // f.jacobian(J, q);
  }

  static LiegroupElement py_value(const DifferentiableFunction& f,
                                  const Configuration_t& q) {
    return f(q);
  }

  static matrix_t py_jacobian(const DifferentiableFunction& f,
                              const Configuration_t& q) {
    matrix_t J(f.outputDerivativeSize(), f.inputDerivativeSize());
    f.jacobian(J, q);
    return J;
  }
};

void exposeDifferentiableFunction() {
  // DocClass (DifferentiableFunction)
  // class_<DifferentiableFunction, DifferentiableFunctionPtr_t,
  // boost::noncopyable>
  class_<DFWrapper, DFWrapper::Ptr_t, boost::noncopyable>(
      "DifferentiableFunction", no_init)
      // Pythonic API
      .def("__str__", &to_str<DifferentiableFunction>)
      .def("__call__", &DFWrapper::py_value)
      .def("J", &DFWrapper::py_jacobian)

      .add_property("ni", &DifferentiableFunction::inputSize)
      .add_property("no", &DifferentiableFunction::outputSize)
      .add_property("ndi", &DifferentiableFunction::inputDerivativeSize)
      .add_property("ndo", &DifferentiableFunction::outputDerivativeSize)

      // C++ API
      .def("value", &DFWrapper::value_wrap, DocClassMethod(value))
      .def("jacobian", &DFWrapper::jacobian_wrap, DocClassMethod(jacobian))

      .def("outputSpace", &DifferentiableFunction::outputSpace,
           DocClassMethod(outputSpace))

      .def("inputSize", &DifferentiableFunction::inputSize,
           DocClassMethod(inputSize))
      .def("outputSize", &DifferentiableFunction::outputSize,
           DocClassMethod(outputSize))
      .def("inputDerivativeSize", &DifferentiableFunction::inputDerivativeSize,
           DocClassMethod(inputDerivativeSize))
      .def("outputDerivativeSize",
           &DifferentiableFunction::outputDerivativeSize,
           DocClassMethod(outputDerivativeSize))
      //;

      // class_<DFWrapper, DFWrapper::Ptr_t, boost::noncopyable,
      // bases<DifferentiableFunction> >
      //("DifferentiableFunctionBase", no_init)
      .def(init<size_type, size_type, size_type, std::string>())
      .def("impl_compute", pure_virtual(&DFWrapper::impl_compute))
      .def("impl_jacobian", pure_virtual(&DFWrapper::impl_jacobian));
}
}  // namespace constraints
}  // namespace pyhpp
