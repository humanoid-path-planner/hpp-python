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
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pyhpp/pinocchio/urdf/fwd.hh>
#include <pyhpp/util.hh>

// #include <pinocchio/bindings/python/fwd.hpp>
// #include <pinocchio/bindings/python/multibody/model.hpp>

// Expose pinocchio model again since the template arguments slightly differ
// with the type exposed by pinocchio bindings.
namespace pinocchio {
namespace python {}  // namespace python
}  // namespace pinocchio

using namespace boost::python;

namespace pyhpp {
namespace pinocchio {

namespace bp = boost::python;
typedef hpp::pinocchio::Model Model;
typedef hpp::pinocchio::Data Data;
typedef hpp::pinocchio::ModelPtr_t ModelPtr_t;
typedef hpp::pinocchio::GeomData GeomData;
typedef hpp::pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;
typedef hpp::pinocchio::GeomModel GeomModel;
typedef hpp::pinocchio::Configuration_t Configuration_t;
typedef hpp::pinocchio::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::pinocchio::size_type size_type;
typedef hpp::pinocchio::Transform3s Transform3s;
typedef hpp::pinocchio::Device Device;
typedef hpp::pinocchio::DevicePtr_t DevicePtr_t;
typedef hpp::pinocchio::GripperPtr_t GripperPtr_t;
typedef hpp::pinocchio::Gripper Gripper;
typedef hpp::pinocchio::Computation_t Computation_t;
typedef hpp::pinocchio::value_type value_type;

using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload, Model::getFrameId,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload, Model::existFrame,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,
                                       Model::addJointFrame, 1, 2)

bool Device_currentConfiguration(Device& d, const Configuration_t& c) {
  return d.currentConfiguration(c);
}

Transform3s getObjectPositionInJoint(const GripperPtr_t& gripper) {
  Transform3s res(gripper->objectPositionInJoint());
  return res;
}

void exposeGripper() {
  class_<Gripper, GripperPtr_t>("Gripper", no_init)
      .def("create", &Gripper::create)
      .staticmethod("create")
      .add_property("localPosition", &getObjectPositionInJoint)
      .add_property(
          "clearance",
          static_cast<value_type (Gripper::*)() const>(&Gripper::clearance),
          static_cast<void (Gripper::*)(const value_type&)>(&Gripper::clearance));
  class_<std::map<std::string, GripperPtr_t> >("GripperMap")
      .def(
          boost::python::map_indexing_suite<std::map<std::string, GripperPtr_t>,
                                            true>());
}
void exposeDevice() {
  enum_<Computation_t>("ComputationFlag")
      .value("JOINT_POSITION", hpp::pinocchio::JOINT_POSITION)
      .value("JACOBIAN", hpp::pinocchio::JACOBIAN)
      .value("VELOCITY", hpp::pinocchio::VELOCITY)
      .value("ACCELERATION", hpp::pinocchio::ACCELERATION)
      .value("COM", hpp::pinocchio::COM)
      .value("COMPUTE_ALL", hpp::pinocchio::COMPUTE_ALL);
  void (Device::*cfk)(int) = &Device::computeForwardKinematics;
  class_<Device, DevicePtr_t, boost::noncopyable>("Device", no_init)
      .def("name", &Device::name, return_value_policy<return_by_value>())
      .def("create", &Device::create)
      .staticmethod("create")
      .def("configSpace", &Device::configSpace,
           return_value_policy<return_by_value>())
      .def("model", static_cast<Model& (Device::*)()>(&Device::model),
           return_internal_reference<>())
      .def("data", static_cast<Data& (Device::*)()>(&Device::data),
           return_internal_reference<>())
      .def("geomData", static_cast<GeomData& (Device::*)()>(&Device::geomData),
           return_internal_reference<>())
      .def("geomModel",
           static_cast<GeomModel& (Device::*)()>(&Device::geomModel),
           return_internal_reference<>())
      .PYHPP_DEFINE_METHOD(Device, configSize)
      .PYHPP_DEFINE_METHOD(Device, numberDof)

      .def("currentConfiguration",
           static_cast<const Configuration_t& (Device::*)() const>(
               &Device::currentConfiguration),
           return_value_policy<return_by_value>())
      .def("currentConfiguration", Device_currentConfiguration)

      .def("computeForwardKinematics", cfk)
      .def("computeFramesForwardKinematics",
           &Device::computeFramesForwardKinematics)
      .def("updateGeometryPlacements", &Device::updateGeometryPlacements);
}
}  // namespace pinocchio
}  // namespace pyhpp
