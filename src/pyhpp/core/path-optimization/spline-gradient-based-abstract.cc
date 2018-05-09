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

#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/path-optimization/linear-constraint.hh>
#include <hpp/core/path-optimization/quadratic-program.hh>
#include <hpp/core/path-optimization/spline-gradient-based-abstract.hh>

#include <pyhpp/stl-pair.hh>
#include <pyhpp/util.hh>
#include <pyhpp/vector-indexing-suite.hh>

using namespace boost::python;

#define ADD_DATA_PROPERTY_READWRITE(TYPE,NAME,DOC)                            \
      def_readwrite (#NAME, &TYPE::NAME, DOC)

#define ADD_DATA_PROPERTY_READONLY_BYVALUE(TYPE,NAME,DOC)                      \
      add_property(#NAME,                                                      \
      make_getter(&TYPE::NAME,return_value_policy<return_by_value>()), DOC)

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
          typedef typename Base::SplineOptimizationData SplineOptimizationData;
          typedef typename Base::SplineOptimizationDatas_t SplineOptimizationDatas_t;

          // using Base::Base;
          SGBWrapper (const Problem& p) : Base (p) {}
          virtual ~SGBWrapper() {}

          PathVectorPtr_t optimize (const PathVectorPtr_t& path)
          {
            override f = this->get_override("optimize");
            if (!f)
              throw std::runtime_error ("optimize not implemented in child class");
            return f (path).as<PathVectorPtr_t>();
          }

          // The following functions are protected methods of SplineGradientBasedAbstract.
          // These wrappers are needed to make them accessible.
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

          void jointBoundConstraint (const Splines_t& splines, LinearConstraint& lc) const
          {
            Base::jointBoundConstraint (splines, lc);
          }

          void addContinuityConstraints (const Splines_t& splines, const size_type maxOrder, const SplineOptimizationDatas_t& ess, LinearConstraint& continuity)
          {
            Base::addContinuityConstraints (splines, maxOrder, ess, continuity);
          }

          PathVectorPtr_t buildPathVector (const Splines_t& splines) const
          {
            return Base::buildPathVector (splines);
          }
      };

      template <int _PolynomeBasis, int _Order>
      void exposeSplineGradientBasedAbstract (const char* name)
      {
        typedef SplineGradientBasedAbstract<_PolynomeBasis, _Order> SGB_t;
        typedef SGBWrapper <_PolynomeBasis, _Order> SGBW_t;
        typedef boost::shared_ptr<SGBW_t> Ptr_t;
        typedef typename SGB_t::Splines_t Splines_t;
        typedef typename SGBW_t::SplineOptimizationData SplineOptimizationData;
        typedef typename SGBW_t::SplineOptimizationDatas_t SplineOptimizationDatas_t;

        scope s =
          class_ <SGBW_t, Ptr_t, bases<PathOptimizer>, boost::noncopyable> (name, init<const Problem&>())
          PYHPP_DEFINE_METHOD (SGB_t, copy         ).staticmethod ("copy")
          PYHPP_DEFINE_METHOD (SGB_t, updateSplines)
          PYHPP_DEFINE_METHOD (SGB_t, interpolate  ).staticmethod ("interpolate")
          PYHPP_DEFINE_METHOD (SGBW_t, appendEquivalentSpline)
          PYHPP_DEFINE_METHOD (SGBW_t, initializePathValidation)
          PYHPP_DEFINE_METHOD (SGBW_t, validatePath)
          PYHPP_DEFINE_METHOD (SGBW_t, jointBoundConstraint)
          PYHPP_DEFINE_METHOD (SGBW_t, addContinuityConstraints)
          PYHPP_DEFINE_METHOD (SGBW_t, buildPathVector)
          ;

        class_ <Splines_t> ("Splines")
          .def (cpp_like_vector_indexing_suite <Splines_t, true> ())
          ;

        class_ <SplineOptimizationData> ("SplineOptimizationData", init<>())
          .def (init<size_type>())
          .def_readwrite ("set", &SplineOptimizationData::set)
          .def_readwrite ("es",  &SplineOptimizationData::es)
          .def_readwrite ("activeParameters", &SplineOptimizationData::activeParameters)
          ;

        class_ <SplineOptimizationDatas_t> ("SplineOptimizationDatas"
            // )
            , init<std::size_t>())
          .def (init<std::size_t, const SplineOptimizationData&>())
          // .def (vector_indexing_suite <SplineOptimizationDatas_t> ())
          .def ("append",      &vector_indexing_suite<SplineOptimizationDatas_t>::append)
          .def ("__len__",     &vector_indexing_suite<SplineOptimizationDatas_t>::size)
          .def ("__getitem__", &vector_indexing_suite<SplineOptimizationDatas_t>::get_item, return_internal_reference<>())
          .def ("__setitem__", &vector_indexing_suite<SplineOptimizationDatas_t>::set_item)
          // This is taken from /usr/include/boost/python/suite/indexing/indexing_suite.hpp when
          // #if BOOST_WORKAROUND(BOOST_MSVC, < 1300)
          // is false
          .def ("__iter__", typename boost::mpl::if_< boost::mpl::bool_<false>, iterator<SplineOptimizationDatas_t> , iterator<SplineOptimizationDatas_t, return_internal_reference<> > >::type())
          ;
      }

      void exposeNonTemplated ()
      {
        using namespace hpp::core::path;

        typedef SGBWrapper <BernsteinBasis, 1> SGBW_t;
        typedef typename SGBW_t::Reports_t Reports_t;

        stl_pair <Reports_t::value_type::first_type, Reports_t::value_type::second_type> ("Report");
        class_ <Reports_t> ("Reports")
          .def (cpp_like_vector_indexing_suite <Reports_t> ())
          ;
      }

      BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LC_reduceConstraint_overload, reduceConstraint, 2, 3)

      void exposeSplineGradientBasedAbstracts()
      {
        using namespace hpp::core::path;
        exposeNonTemplated ();
        exposeSplineGradientBasedAbstract<BernsteinBasis, 1> ("SplineGradientBasedAbstractB1");
        exposeSplineGradientBasedAbstract<BernsteinBasis, 3> ("SplineGradientBasedAbstractB3");
      }

      void exposeLinearConstraint()
      {
        class_ <LinearConstraint> ("LinearConstraint", init<size_type, size_type>())
          PYHPP_DEFINE_METHOD (LinearConstraint, concatenate)
          PYHPP_DEFINE_METHOD (LinearConstraint, decompose)
          PYHPP_DEFINE_METHOD (LinearConstraint, computeRank)
          .def ("reduceConstraint", &LinearConstraint::reduceConstraint,
              LC_reduceConstraint_overload (args ("toReduce", "reduced", "computeRank")))
          PYHPP_DEFINE_METHOD (LinearConstraint, computeSolution)
          PYHPP_DEFINE_METHOD (LinearConstraint, isSatisfied)
          PYHPP_DEFINE_METHOD (LinearConstraint, addRows)
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, J    , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, b    , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, rank , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, PK   , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, xStar, "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (LinearConstraint, xSol , "")
          ;
      }

      struct QPWrapper {
        static void setH (QuadraticProgram& qp, const matrix_t& H) { qp.H = H; }
        static void setb (QuadraticProgram& qp, const vector_t& b) { qp.b = b; }
      };

      void exposeQuadraticProblem()
      {
        class_ <QuadraticProgram> ("QuadraticProgram", init<size_type>())
          .def (init<const QuadraticProgram&, const LinearConstraint&>())
          PYHPP_DEFINE_METHOD (QuadraticProgram, addRows)
          PYHPP_DEFINE_METHOD (QuadraticProgram, reduced)
          PYHPP_DEFINE_METHOD (QuadraticProgram, decompose)
          .def ("solve", static_cast<void (QuadraticProgram::*) ()>
              (&QuadraticProgram::solve))

          PYHPP_DEFINE_METHOD (QuadraticProgram, computeLLT)
          .def ("solve", static_cast<double (QuadraticProgram::*) (const LinearConstraint&,const LinearConstraint&)>
              (&QuadraticProgram::solve))

          .add_property ("H",
              make_getter (&QuadraticProgram::H, return_value_policy<return_by_value>()),
              &QPWrapper::setH, 
              "")
          .add_property ("b",
              make_getter (&QuadraticProgram::b, return_value_policy<return_by_value>()),
              &QPWrapper::setb, 
              "")
          .ADD_DATA_PROPERTY_READWRITE (QuadraticProgram, bIsZero         , "")

          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, llt             , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, trace           , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, activeConstraint, "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, activeSetSize   , "")

          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, dec  , "")
          .ADD_DATA_PROPERTY_READONLY_BYVALUE (QuadraticProgram, xStar, "")
          ;
      }
    }
  }
}
