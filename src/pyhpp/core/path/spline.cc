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

#include <pyhpp/core/path/fwd.hh>

#include <boost/python.hpp>

#include <hpp/core/path/spline.hh>

#include <pyhpp/util.hh>
#include <hpp/python/config.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    namespace path {
      using hpp::core::Path;
      using namespace hpp::core::path;

      template <int _PolynomeBasis, int _Order>
      void exposeSpline (const char* name)
      {
        typedef Spline<_PolynomeBasis, _Order> S_t;
        typedef typename S_t::Ptr_t Ptr_t;

        scope s =
          class_ <S_t, Ptr_t, bases<Path>, boost::noncopyable> (name, no_init)
          ;

        enum_ <int> ("")
          .value ("PolynomeBasis", S_t::PolynomeBasis)
          .value ("Order", S_t::Order)
          .value ("NbCoeffs", S_t::NbCoeffs)
          .export_values()
          ;
      }

      void exposeSplines()
      {
        exposeSpline<BernsteinBasis, 1> ("SplineB1");
        exposeSpline<BernsteinBasis, 3> ("SplineB3");
      }
    }
  }
}
