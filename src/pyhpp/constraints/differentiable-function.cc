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

#include <hpp/constraints/differentiable-function.hh>

using namespace boost::python;

namespace pyhpp {
  namespace constraints {
    using namespace hpp::constraints;

    LiegroupElement DifferentiableFunction_value (
        const DifferentiableFunction& f, const Configuration_t& q)
    {
      return f(q);
    }

    // void DifferentiableFunction_jacobian (
        // const DifferentiableFunction& f, matrix_t& J, const Configuration_t& q)
    // {
      // f.jacobian(J, q);
    // }
    matrix_t DifferentiableFunction_jacobian (
        const DifferentiableFunction& f, const Configuration_t& q)
    {
      matrix_t J (f.outputDerivativeSize(), f.inputDerivativeSize());
      f.jacobian(J, q);
      return J;
    }

    void exposeDifferentiableFunction ()
    {
      class_<DifferentiableFunction, DifferentiableFunctionPtr_t, boost::noncopyable> ("DifferentiableFunction", no_init)
        .def ("__call__", &DifferentiableFunction_value)
        .def ("value",    &DifferentiableFunction_value)
        .def ("jacobian", &DifferentiableFunction_jacobian)

        .add_property ("inputSize", &DifferentiableFunction::inputSize)
        ;
    }
  }
}
