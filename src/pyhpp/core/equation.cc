// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <pyhpp/core/fwd.hh>

#include <boost/python.hpp>

#include <eigenpy/eigenpy.hpp>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/core/equation.hh>
#include <hpp/core/numerical-constraint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/locked-joint.hh>

#include <pyhpp/util.hh>
#include <pyhpp/vector-indexing-suite.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    using namespace hpp::core;

    struct EWrapper {
      static void rightHandSideFromConfig (Equation* eq, const vector_t& q)
      {
        eq->rightHandSideFromConfig (q);
      }
      static void setRightHandSide (Equation& eq, const vector_t& rhs)
      {
        eq.rightHandSide (rhs);
      }
      static vector_t getRightHandSide (const Equation& eq)
      {
        return eq.rightHandSide ();
      }
      static EquationPtr_t copy (const Equation* e) { return e->copy(); }
    };

    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CP_add_overloads, add, 1, 3)

    void exposeEquation ()
    {
      class_ <Equation, EquationPtr_t, boost::noncopyable>
        ("Equation", no_init)
        PYHPP_DEFINE_GETTER_SETTER_INTERNAL_REF (Equation, comparisonType, const ComparisonTypes_t&)
        .def ("rightHandSideFromConfig", &EWrapper::rightHandSideFromConfig)
        .def ("rightHandSide", &EWrapper::getRightHandSide)
        .def ("rightHandSide", &EWrapper::setRightHandSide)
        PYHPP_DEFINE_METHOD (Equation, copy)
        ;

      class_ <NumericalConstraint, NumericalConstraintPtr_t, boost::noncopyable, bases<Equation> >
        ("NumericalConstraint", no_init)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (NumericalConstraint, functionPtr)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (NumericalConstraint, value)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (NumericalConstraint, jacobian)
        ;

      class_ <NumericalConstraints_t> ("NumericalConstraints")
        .def (cpp_like_vector_indexing_suite <NumericalConstraints_t> ())
        ;

      class_ <ExplicitNumericalConstraint, ExplicitNumericalConstraintPtr_t, boost::noncopyable, bases<NumericalConstraint> >
        ("ExplicitNumericalConstraint", no_init)
        PYHPP_DEFINE_METHOD (ExplicitNumericalConstraint, explicitFunction)
        PYHPP_DEFINE_METHOD (ExplicitNumericalConstraint, outputFunction)
        PYHPP_DEFINE_METHOD (ExplicitNumericalConstraint, outputFunctionInverse)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (ExplicitNumericalConstraint,  inputConf)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (ExplicitNumericalConstraint,  inputVelocity)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (ExplicitNumericalConstraint, outputConf)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (ExplicitNumericalConstraint, outputVelocity)
        ;

      class_ <LockedJoint, LockedJointPtr_t, boost::noncopyable, bases<ExplicitNumericalConstraint> >
        ("LockedJoint", no_init)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (LockedJoint, jointName)
        PYHPP_DEFINE_METHOD (LockedJoint, configSize)
        PYHPP_DEFINE_METHOD (LockedJoint, numberDof)
        PYHPP_DEFINE_METHOD (LockedJoint, rankInConfiguration)
        PYHPP_DEFINE_METHOD (LockedJoint, rankInVelocity)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (LockedJoint, configSpace)
        // PYHPP_DEFINE_METHOD (LockedJoint, value)
        PYHPP_DEFINE_METHOD (LockedJoint,  inputConf)
        PYHPP_DEFINE_METHOD (LockedJoint,  inputVelocity)
        PYHPP_DEFINE_METHOD (LockedJoint, outputConf)
        PYHPP_DEFINE_METHOD (LockedJoint, outputVelocity)
        ;

      // TODO LockedJoints_t is not a vector but a list...
      // class_ <LockedJoints_t> ("LockedJoints")
        // .def (cpp_like_vector_indexing_suite <LockedJoints_t> ())
        // ;
    }
  }
}
