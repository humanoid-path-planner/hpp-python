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

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    namespace pathOptimization {
      using namespace hpp::core;
      using namespace hpp::core::pathOptimization;

      template <int _PolynomeBasis, int _Order> class SGBWrapper :
        public SplineGradientBasedAbstract<_PolynomeBasis, _Order>,
        public wrapper <SplineGradientBasedAbstract<_PolynomeBasis, _Order> >
      {
        public:
          typedef SplineGradientBasedAbstract<_PolynomeBasis, _Order> Base;
          typedef typename Base::Splines_t Splines_t;
          typedef typename Base::Reports_t Reports_t;

          void appendEquivalentSpline (const PathVectorPtr_t& pv, Splines_t& ss) const
          {
            Base::appendEquivalentSpline (pv, ss);
          }

          void initializePathValidation(const Splines_t& splines)
          {
            override f = this->get_override("initializePathValidation");
            if (!f) Base::initializePathValidation (splines);
            else    f (splines);
          }

          Reports_t validatePath (const Splines_t& splines, bool stopAtFirst) const
          {
            return Base::validatePath (splines, stopAtFirst);
          }

          PathVectorPtr_t buildPathVector (const Splines_t& splines) const
          {
            return buildPathVector (splines);
          }

          PathVectorPtr_t optimize (const PathVectorPtr_t& path) const
          {
            override f = this->get_override("optimize");
            if (!f)
              throw std::runtime_error ("optimize not implemented in child class");
            return f (path);
          }
      };

      template <int _PolynomeBasis, int _Order>
      void exposeSplineGradientBasedAbstract (const char* name)
      {
        typedef SplineGradientBasedAbstract<_PolynomeBasis, _Order> SGB_t;
        typedef SGBWrapper <_PolynomeBasis, _Order> SGBW_t;
        typedef boost::shared_ptr<SGBW_t> Ptr_t;
        typedef typename SGB_t::Splines_t Splines_t;
        typedef typename SGBW_t::Reports_t Reports_t;

        scope s =
          class_ <SGBW_t, Ptr_t, bases<PathOptimizer>, boost::noncopyable> (name, no_init)
          PYHPP_DEFINE_METHOD (SGBW_t, appendEquivalentSpline)
          PYHPP_DEFINE_METHOD (SGBW_t, initializePathValidation)
          PYHPP_DEFINE_METHOD (SGBW_t, validatePath)
          ;

        class_ <Splines_t> ("Splines")
          .def (vector_indexing_suite <Splines_t> ())
          ;

        class_ <Reports_t> ("Reports")
          .def (vector_indexing_suite <Reports_t> ())
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
