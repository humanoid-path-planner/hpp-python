//
// Copyright (c) 2018 - 2023 CNRS
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
#include <hpp/constraints/generic-transformation.hh>
#include <pyhpp/constraints/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

template <typename GT_t>
typename GT_t::Ptr_t AbsoluteGenericTransformation_create(
    const std::string& name, const DevicePtr_t& robot,
    const pinocchio::JointIndex& j2, const Transform3s& frame2,
    const Transform3s& frame1,
    std::vector<bool> mask  // TODO Add default argument
) {
  JointConstPtr_t joint2(new hpp::pinocchio::Joint(robot, j2));
  return GT_t::create(name, robot, joint2, frame2, frame1, mask);
}

template <typename GT_t>
void exposeAbsoluteGenericTransformation(const char* name) {
  // BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GT_t_create_overload, create, 5, 6)

  class_<GT_t, bases<DifferentiableFunction>, typename GT_t::Ptr_t,
         boost::noncopyable>(name, no_init)
      .def("create", &AbsoluteGenericTransformation_create<GT_t>,
           args("name", "device", "joint2", "frame2", "frame1", "mask"))
      .staticmethod("create");
}

template <typename GT_t>
typename GT_t::Ptr_t RelativeGenericTransformation_create(
    const std::string& name, const DevicePtr_t& robot,
    const pinocchio::JointIndex& j1, const pinocchio::JointIndex& j2,
    const Transform3s& frame1, const Transform3s& frame2,
    std::vector<bool> mask  // TODO Add default argument
) {
  JointConstPtr_t joint1(new hpp::pinocchio::Joint(robot, j1));
  JointConstPtr_t joint2(new hpp::pinocchio::Joint(robot, j2));
  return GT_t::create(name, robot, joint1, joint2, frame1, frame2, mask);
}

template <typename GT_t>
void exposeRelativeGenericTransformation(const char* name) {
  class_<GT_t, bases<DifferentiableFunction>, typename GT_t::Ptr_t,
         boost::noncopyable>(name, no_init)
      .def("create", &RelativeGenericTransformation_create<GT_t>,
           args("name", "device", "joint1", "joint2", "frame1", "frame2",
                "mask"))
      .staticmethod("create");
}

void exposeGenericTransformations() {
  exposeAbsoluteGenericTransformation<Position>("Position");
  exposeAbsoluteGenericTransformation<Orientation>("Orientation");
  exposeAbsoluteGenericTransformation<Transformation>("Transformation");
  exposeAbsoluteGenericTransformation<RelativePosition>("RelativePosition");
  exposeAbsoluteGenericTransformation<RelativeOrientation>(
      "RelativeOrientation");
  exposeAbsoluteGenericTransformation<RelativeTransformation>(
      "RelativeTransformation");
}
}  // namespace constraints
}  // namespace pyhpp
