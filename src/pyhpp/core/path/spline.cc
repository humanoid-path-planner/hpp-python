//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <hpp/core/path/spline.hh>
#include <hpp/python/config.hh>
#include <pyhpp/core/path/fwd.hh>
#include <pyhpp/ref.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::core::path)

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

  static void parameterDerivativeCoefficients(const S_t& s, vectorRef_t res,
                                              const value_type& t) {
    vector_t _res(res);
    s.parameterDerivativeCoefficients(_res, t);
    res = _res;
  }

  static void parameterIntegrate(S_t& s, const vector_t& dParam) {
    s.parameterIntegrate(dParam);
  }

  static void squaredNormIntegralDerivative(const S_t& s, const size_type order,
                                            vectorRef_t res) {
    vector_t _res(res);
    s.squaredNormIntegralDerivative(order, _res);
    res = _res;
  }

  static void basisFunctionDerivative(const S_t& s, const size_type order,
                                      const value_type& u, vectorRef_t res) {
    vector_t _res(res);
    s.basisFunctionDerivative(order, u, _res);
    res = _res;
  }

  static void squaredNormBasisFunctionIntegral(const S_t& s,
                                               const size_type order,
                                               matrixRef_t res) {
    matrix_t _res(res);
    s.squaredNormBasisFunctionIntegral(order, _res);
    res = _res;
  }

  static vector_t rowParameters(const S_t& s) {
    return vector_t(s.rowParameters());
  }

  static vector_t py_parameterDerivativeCoefficients(const S_t& s,
                                                     const value_type& t) {
    typename S_t::BasisFunctionVector_t res;
    s.parameterDerivativeCoefficients(res, t);
    return res;
  }

  static void expose(const char* name) {
    scope s =
        class_<S_t, Ptr_t, bases<Path>, boost::noncopyable>(name, no_init)
            .def("parameterDerivativeCoefficients",
                 &SplineWrapper::py_parameterDerivativeCoefficients)

            .PYHPP_DEFINE_METHOD(S_t, parameterSize)
            .PYHPP_DEFINE_METHOD(SplineWrapper, parameterDerivativeCoefficients)
            .PYHPP_DEFINE_METHOD(SplineWrapper, parameterIntegrate)
            .PYHPP_DEFINE_METHOD(S_t, squaredNormIntegral)
            .PYHPP_DEFINE_METHOD(SplineWrapper, squaredNormIntegralDerivative)
            .PYHPP_DEFINE_METHOD(SplineWrapper, basisFunctionDerivative)
            .PYHPP_DEFINE_METHOD(SplineWrapper,
                                 squaredNormBasisFunctionIntegral)
            .PYHPP_DEFINE_METHOD(SplineWrapper, rowParameters);

    enum_<int>("")
        .value("PolynomeBasis", S_t::PolynomeBasis)
        .value("Order", S_t::Order)
        .value("NbCoeffs", S_t::NbCoeffs)
        .export_values();
  }
};

void exposeSplines() {
  SplineWrapper<BernsteinBasis, 1>::expose("SplineB1");
  SplineWrapper<BernsteinBasis, 3>::expose("SplineB3");
}
}  // namespace path
}  // namespace core
}  // namespace pyhpp
