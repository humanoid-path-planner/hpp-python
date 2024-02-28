//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/fwd.hpp>
#include <pyhpp/pinocchio/urdf/fwd.hh>
#include <pyhpp/ref.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace pinocchio {
using namespace hpp::pinocchio;

struct LgSWrapper {
  static LiegroupSpacePtr_t itimes(LiegroupSpacePtr_t l,
                                   const LiegroupSpacePtr_t& r) {
    return *l *= r;
  }
  static LiegroupSpacePtr_t times(LiegroupSpacePtr_t l,
                                  const LiegroupSpacePtr_t& r) {
    return l * r;
  }
  static void dIntegrate_dq(DerivativeProduct side, LiegroupElementConstRef q,
                            const vector_t& v, matrixOut_t J) {
    enum_<hpp::pinocchio::DerivativeProduct>("DerivativeProduct")
        .value("DerivativeTimesInput", DerivativeTimesInput)
        .value("InputTimesDerivative", InputTimesDerivative);

    LiegroupSpacePtr_t ls(q.space());
    matrix_t _J(J);
    switch (side) {
      case DerivativeTimesInput:
        ls->dIntegrate_dq<DerivativeTimesInput>(q, v, _J);
        break;
      case InputTimesDerivative:
        ls->dIntegrate_dq<InputTimesDerivative>(q, v, _J);
        break;
      default:
        abort();
    }
    J = _J;
  }

  static void dIntegrate_dv(DerivativeProduct side, LiegroupElementConstRef q,
                            const vector_t& v, matrixOut_t J) {
    enum_<hpp::pinocchio::DerivativeProduct>("DerivativeProduct")
        .value("DerivativeTimesInput", DerivativeTimesInput)
        .value("InputTimesDerivative", InputTimesDerivative);

    LiegroupSpacePtr_t ls(q.space());
    matrix_t _J(J);
    switch (side) {
      case DerivativeTimesInput:
        ls->dIntegrate_dv<DerivativeTimesInput>(q, v, _J);
        break;
      case InputTimesDerivative:
        ls->dIntegrate_dv<InputTimesDerivative>(q, v, _J);
        break;
      default:
        abort();
    }
    J = _J;
  }

  static void dDifference_dq0(const LiegroupSpace& ls, DerivativeProduct side,
                              const vector_t& q0, const vector_t& q1,
                              matrixOut_t J) {
    enum_<hpp::pinocchio::DerivativeProduct>("DerivativeProduct")
        .value("DerivativeTimesInput", DerivativeTimesInput)
        .value("InputTimesDerivative", InputTimesDerivative);

    matrix_t _J(J);
    switch (side) {
      case DerivativeTimesInput:
        ls.dDifference_dq0<DerivativeTimesInput>(q0, q1, _J);
        break;
      case InputTimesDerivative:
        ls.dDifference_dq0<InputTimesDerivative>(q0, q1, _J);
        break;
      default:
        abort();
    }
    J = _J;
  }

  static void dDifference_dq1(const LiegroupSpace& ls, DerivativeProduct side,
                              const vector_t& q0, const vector_t& q1,
                              matrixOut_t J) {
    enum_<hpp::pinocchio::DerivativeProduct>("DerivativeProduct")
        .value("DerivativeTimesInput", DerivativeTimesInput)
        .value("InputTimesDerivative", InputTimesDerivative);

    matrix_t _J(J);
    switch (side) {
      case DerivativeTimesInput:
        ls.dDifference_dq1<DerivativeTimesInput>(q0, q1, _J);
        break;
      case InputTimesDerivative:
        ls.dDifference_dq1<InputTimesDerivative>(q0, q1, _J);
        break;
      default:
        abort();
    }
    J = _J;
  }
};
struct LgEWrapper {
  static vector_t vector_wrap_read(const LiegroupElement& lge) {
    return lge.vector();
  }
  static void vector_wrap_write(LiegroupElement& lge, const vector_t& v) {
    lge.vector() = v;
  }
};

struct LgERWrapper {
  static vectorOut_t vector_wrap_read(const LiegroupElementRef& lge) {
    return lge.vector();
  }
  static void vector_wrap_write(LiegroupElementRef& lge, const vector_t& v) {
    lge.vector() = v;
  }
};

void exposeLiegroup() {
  class_<LiegroupSpace, LiegroupSpacePtr_t, boost::noncopyable>("LiegroupSpace",
                                                                no_init)
      .def("__str__", &to_str_from_operator<LiegroupSpace>)
      .def("name", &LiegroupSpace::name, return_value_policy<return_by_value>())
      .PYHPP_DEFINE_METHOD(LiegroupSpace, Rn)
      .staticmethod("Rn")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, R1)
      .staticmethod("R1")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, R2)
      .staticmethod("R2")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, R3)
      .staticmethod("R3")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, SE2)
      .staticmethod("SE2")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, SE3)
      .staticmethod("SE3")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, R2xSO2)
      .staticmethod("R2xSO2")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, R3xSO3)
      .staticmethod("R3xSO3")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, empty)
      .staticmethod("empty")
      .PYHPP_DEFINE_METHOD(LiegroupSpace, mergeVectorSpaces)
      .PYHPP_DEFINE_METHOD(LgSWrapper, dIntegrate_dq)
      .PYHPP_DEFINE_METHOD(LgSWrapper, dIntegrate_dv)
      .PYHPP_DEFINE_METHOD(LgSWrapper, dDifference_dq0)
      .PYHPP_DEFINE_METHOD(LgSWrapper, dDifference_dq1)
      .def(self == self)
      .def(self != self)
      // Operation on shared pointers...
      .def("__imul__", &LgSWrapper::itimes)
      .def("__mul__", &LgSWrapper::times);

  class_<LiegroupElement>("LiegroupElement",
                          init<const vector_t&, const LiegroupSpacePtr_t&>())
      .def(init<const LiegroupSpacePtr_t&>())
      // Pythonic API
      .def("__str__", &to_str_from_operator<LiegroupElement>)
      .add_property("v", &LgEWrapper::vector_wrap_read,
                    &LgEWrapper::vector_wrap_write)

      // C++ API
      .def("vector",
           static_cast<const vector_t& (LiegroupElement::*)() const>(
               &LiegroupElement::vector),
           return_value_policy<return_by_value>())
      .def("space", &LiegroupElement::space,
           return_value_policy<return_by_value>())
      .def(self - self)
      .def(self + vector_t());

  class_<LiegroupElementRef>("LiegroupElementRef",
                             init<vectorOut_t, LiegroupSpacePtr_t&>())
      // Pythonic API
      .def("__str__", &to_str_from_operator<LiegroupElementRef>)
      .add_property("v", &LgERWrapper::vector_wrap_read,
                    &LgERWrapper::vector_wrap_write)

      // C++ API
      .def("vector",
           static_cast<const vectorOut_t& (LiegroupElementRef::*)() const>(
               &LiegroupElementRef::vector),
           return_value_policy<return_by_value>())
      .def("space", &LiegroupElementRef::space,
           return_value_policy<return_by_value>())
      .def(self - self)
      .def(self + vector_t());
}
}  // namespace pinocchio
}  // namespace pyhpp
