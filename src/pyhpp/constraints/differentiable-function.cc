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

#include <pyhpp/constraints/fwd.hh>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/eigenpy.hpp>

#include <hpp/constraints/differentiable-function.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace constraints {
    using namespace hpp::constraints;

    LiegroupElement DifferentiableFunction_call (
        const DifferentiableFunction& f, const Configuration_t& q)
    {
      return f(q);
    }

    void DifferentiableFunction_value (
        const DifferentiableFunction& f, LiegroupElement& lge, const Configuration_t& q)
    {
      f.value(lge, q);
    }

    // NOTE: eigenpy::Ref<const vector_t> is not binded so far. It is not very
    // useful since it cannot be used to return a value.
    void DifferentiableFunction_jacobian_wrap (
        const DifferentiableFunction& f,
        eigenpy::Ref<matrix_t> J,
        const Configuration_t& q)
        // const eigenpy::Ref<const vector_t>& q)
    {
      matrix_t Jtmp (f.outputDerivativeSize(), f.inputDerivativeSize());
      f.jacobian(Jtmp, q);
      J = Jtmp;
      // f.jacobian(J, q);
    }

    matrix_t DifferentiableFunction_jacobian (
        const DifferentiableFunction& f, 
        const Configuration_t& q)
    {
      matrix_t J (f.outputDerivativeSize(), f.inputDerivativeSize());
      f.jacobian(J, q);
      return J;
    }

    void exposeDifferentiableFunction ()
    {
      class_<DifferentiableFunction, DifferentiableFunctionPtr_t, boost::noncopyable> ("DifferentiableFunction", no_init)
        // Pythonic API
        .def ("__str__", &to_str<DifferentiableFunction>)
        .def ("__call__", &DifferentiableFunction_call)
        .def ("J",        &DifferentiableFunction_jacobian)

        .add_property ("ni",  &DifferentiableFunction::inputSize)
        .add_property ("no", &DifferentiableFunction::outputSize)
        .add_property ("ndi",  &DifferentiableFunction::inputDerivativeSize)
        .add_property ("ndo", &DifferentiableFunction::outputDerivativeSize)

        // C++ API
        .def ("value",    &DifferentiableFunction_value)
        .def ("jacobian", &DifferentiableFunction_jacobian_wrap)

        .def ("outputSpace", &DifferentiableFunction::outputSpace)

        .def ("inputSize",  &DifferentiableFunction::inputSize)
        .def ("outputSize", &DifferentiableFunction::outputSize)
        .def ("inputDerivativeSize",  &DifferentiableFunction::inputDerivativeSize)
        .def ("outputDerivativeSize", &DifferentiableFunction::outputDerivativeSize)
        ;
    }
  }
}
