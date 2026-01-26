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
#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/fwd.hpp>
#include <pyhpp/pinocchio/urdf/fwd.hh>
#include <pyhpp/ref.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::pinocchio)

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
  // DocClass(LiegroupSpace)
  class_<LiegroupSpace, LiegroupSpacePtr_t, boost::noncopyable>("LiegroupSpace",
                                                                no_init)
      .def("__str__", &to_str_from_operator<LiegroupSpace>)
      .def("name", &LiegroupSpace::name, return_value_policy<return_by_value>(),
           DocClassMethod(name))
      .def("Rn", &LiegroupSpace::Rn, DocClassMethod(Rn))
      .staticmethod("Rn")
      .def("R1", &LiegroupSpace::R1, DocClassMethod(R1))
      .staticmethod("R1")
      .def("R2", &LiegroupSpace::R2, DocClassMethod(R2))
      .staticmethod("R2")
      .def("R3", &LiegroupSpace::R3, DocClassMethod(R3))
      .staticmethod("R3")
      .def("SE2", &LiegroupSpace::SE2, DocClassMethod(SE2))
      .staticmethod("SE2")
      .def("SE3", &LiegroupSpace::SE3, DocClassMethod(SE3))
      .staticmethod("SE3")
      .def("R2xSO2", &LiegroupSpace::R2xSO2, DocClassMethod(R2xSO2))
      .staticmethod("R2xSO2")
      .def("R3xSO3", &LiegroupSpace::R3xSO3, DocClassMethod(R3xSO3))
      .staticmethod("R3xSO3")
      .def("empty", &LiegroupSpace::empty, DocClassMethod(empty))
      .staticmethod("empty")
      .def("mergeVectorSpaces", &LiegroupSpace::mergeVectorSpaces,
           DocClassMethod(mergeVectorSpaces))
      .def("dIntegrate_dq", &LgSWrapper::dIntegrate_dq)
      .def("dIntegrate_dv", &LgSWrapper::dIntegrate_dv)
      .def("dDifference_dq0", &LgSWrapper::dDifference_dq0)
      .def("dDifference_dq1", &LgSWrapper::dDifference_dq1)
      .def(self == self)
      .def(self != self)
      // Operation on shared pointers...
      .def("__imul__", &LgSWrapper::itimes)
      .def("__mul__", &LgSWrapper::times);

  // DocClass(LiegroupElement)
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
           return_value_policy<return_by_value>(), DocClassMethod(vector))
      .def("space", &LiegroupElement::space,
           return_value_policy<return_by_value>(), DocClassMethod(space))
      .def(self - self)
      .def(self + vector_t());

  // DocClass(LiegroupElementRef)
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
           return_value_policy<return_by_value>(), DocClassMethod(vector))
      .def("space", &LiegroupElementRef::space,
           return_value_policy<return_by_value>(), DocClassMethod(space))
      .def(self - self)
      .def(self + vector_t());
}
}  // namespace pinocchio
}  // namespace pyhpp
