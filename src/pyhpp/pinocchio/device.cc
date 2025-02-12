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

Transform3s getObjectPositionInJoint(const GripperPtr_t& gripper)
{
  Transform3s res(gripper->objectPositionInJoint());
  return res;
}

void exposeGripper() {
  class_<Gripper, GripperPtr_t>("Gripper", no_init)
    .def("create", &Gripper::create)
    .staticmethod("create")
    .add_property("localPosition", &getObjectPositionInJoint);
  class_< std::map<std::string, GripperPtr_t> >("GripperMap")
    .def(boost::python::map_indexing_suite< std::map<std::string, GripperPtr_t>, true >());
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
