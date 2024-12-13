//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
