//
// Copyright (c) 2025 CNRS
// Authors: Paul Sardin
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
#include <hpp/constraints/comparison-types.hh>
#include <hpp/constraints/fwd.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup-element.hh>
#include <pyhpp/constraints/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

LockedJointPtr_t createLockedJoint(const DevicePtr_t& robot,
                                   const char* jointName,
                                   const vector_t& config) {
  try {
    // Get robot in hppPlanner object.
    JointPtr_t joint = robot->getJointByName(jointName);
    LiegroupElement lge(config, joint->configurationSpace());
    LockedJointPtr_t lockedJoint(LockedJoint::create(joint, lge));
    return lockedJoint;
  } catch (const std::exception& exc) {
    throw std::runtime_error(exc.what());
  }
}

LockedJointPtr_t createLockedJointWithComp(const DevicePtr_t& robot,
                                           const char* jointName,
                                           const vector_t& config,
                                           const ComparisonTypes_t& comp) {
  try {
    // Get robot in hppPlanner object.
    JointPtr_t joint = robot->getJointByName(jointName);
    LiegroupElement lge(config, joint->configurationSpace());
    LockedJointPtr_t lockedJoint(LockedJoint::create(joint, lge));
    lockedJoint->comparisonType(comp);
    return lockedJoint;
  } catch (const std::exception& exc) {
    throw std::runtime_error(exc.what());
  }
}

void exposeLockedJoint() {
  class_<LockedJoint, bases<Implicit>, LockedJointPtr_t, boost::noncopyable>(
      "LockedJoint", no_init)
      .def("__init__", make_constructor(&createLockedJoint))
      .def("__init__", make_constructor(&createLockedJointWithComp));
}
}  // namespace constraints
}  // namespace pyhpp
