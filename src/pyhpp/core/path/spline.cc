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

#include <pyhpp/ref.hh>
#include <pyhpp/util.hh>
#include <hpp/python/config.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    namespace path {
      using namespace hpp::core;
      using namespace hpp::core::path;

      template <int _PolynomeBasis, int _Order>
      struct SplineWrapper {
        typedef Spline<_PolynomeBasis, _Order> S_t;
        typedef typename S_t::Ptr_t Ptr_t;

        static void parameterDerivativeCoefficients (const S_t& s,
            vectorRef_t res, const value_type& t)
        {
          vector_t _res(res);
          s.parameterDerivativeCoefficients(_res, t);
          res = _res;
        }

        static void parameterIntegrate (S_t& s, const vector_t& dParam)
        {
          s.parameterIntegrate (dParam);
        }

        static void squaredNormIntegralDerivative (const S_t& s,
            const size_type order, vectorRef_t res)
        {
          vector_t _res(res);
          s.squaredNormIntegralDerivative(order, _res);
          res = _res;
        }

        static void basisFunctionDerivative (const S_t& s,
            const size_type order, const value_type& u, vectorRef_t res)
        {
          vector_t _res(res);
          s.basisFunctionDerivative (order, u, _res);
          res = _res;
        }

        static void squaredNormBasisFunctionIntegral (const S_t& s,
            const size_type order, matrixRef_t res)
        {
          matrix_t _res (res);
          s.squaredNormBasisFunctionIntegral (order, _res);
          res = _res;
        }

        static vector_t rowParameters (const S_t& s) { return vector_t(s.rowParameters()); }

        static vector_t py_parameterDerivativeCoefficients (const S_t& s,
            const value_type& t)
        {
          typename S_t::BasisFunctionVector_t res;
          s.parameterDerivativeCoefficients(res, t);
          return res;
        }

        static void expose (const char* name)
        {
          scope s =
            class_ <S_t, Ptr_t, bases<Path>, boost::noncopyable> (name, no_init)
            .def ("parameterDerivativeCoefficients", &SplineWrapper::py_parameterDerivativeCoefficients)

            PYHPP_DEFINE_METHOD (S_t, parameterSize)
            PYHPP_DEFINE_METHOD (SplineWrapper, parameterDerivativeCoefficients)
            PYHPP_DEFINE_METHOD (SplineWrapper, parameterIntegrate)
            PYHPP_DEFINE_METHOD (S_t, squaredNormIntegral)
            PYHPP_DEFINE_METHOD (SplineWrapper, squaredNormIntegralDerivative)
            PYHPP_DEFINE_METHOD (SplineWrapper, basisFunctionDerivative)
            PYHPP_DEFINE_METHOD (SplineWrapper, squaredNormBasisFunctionIntegral)
            PYHPP_DEFINE_METHOD (SplineWrapper, rowParameters)
            ;

          enum_ <int> ("")
            .value ("PolynomeBasis", S_t::PolynomeBasis)
            .value ("Order", S_t::Order)
            .value ("NbCoeffs", S_t::NbCoeffs)
            .export_values()
            ;
        }
      };

      void exposeSplines()
      {
        SplineWrapper<BernsteinBasis, 1>::expose ("SplineB1");
        SplineWrapper<BernsteinBasis, 3>::expose ("SplineB3");
      }
    }
  }
}
