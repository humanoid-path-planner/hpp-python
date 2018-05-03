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

#include <pyhpp/core/path-optimization/fwd.hh>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <hpp/core/path-optimization/spline-gradient-based-abstract.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    namespace pathOptimization {
      using hpp::core::PathOptimizer;
      using namespace hpp::core::pathOptimization;

      template <int _PolynomeBasis, int _Order>
      void exposeSplineGradientBasedAbstract (const char* name)
      {
        typedef SplineGradientBasedAbstract<_PolynomeBasis, _Order> SGB_t;
        typedef boost::shared_ptr<SGB_t> Ptr_t;

        scope s =
          class_ <SGB_t, Ptr_t, bases<PathOptimizer>, boost::noncopyable> (name, no_init)
          ;

        typedef typename SGB_t::Splines_t Splines_t;
        class_ <Splines_t> ("Splines")
          .def (vector_indexing_suite <Splines_t> ())
          ;
      }

      void exposeSplineGradientBasedAbstracts()
      {
        using namespace hpp::core::path;
        exposeSplineGradientBasedAbstract<BernsteinBasis, 1> ("SplineGradientBasedAbstractB1");
        exposeSplineGradientBasedAbstract<BernsteinBasis, 3> ("SplineGradientBasedAbstractB3");
      }
    }
  }
}
