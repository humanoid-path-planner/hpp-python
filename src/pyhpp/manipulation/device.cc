//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
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
#include <pyhpp/util.hh>

#include <../src/pyhpp/manipulation/device.hh>

namespace pyhpp {
namespace manipulation {
using namespace boost::python;

Device::Device(const hpp::manipulation::DevicePtr_t& object)
{
  obj = object;
}
Device::Device(const std::string& name) : obj(hpp::manipulation::Device::create(name))
{
}
// Methods from hpp::pinocchio::Device
const std::string& Device::name() const
{
  return obj->name();
}
const LiegroupSpacePtr_t& Device::configSpace() const
{
  return obj->configSpace();
}
Model& Device::model()
{
  return obj->model();
}
Data& Device::data()
{
  return obj->data();
}
GeomData& Device::geomData()
{
  return obj->geomData();
}
GeomModel& Device::geomModel()
{
  return obj->geomModel();
}
size_type Device::configSize() const
{
  return obj->configSize();
}
size_type Device::numberDof() const
{
  return obj->numberDof();
}
const Configuration_t& Device::currentConfiguration() const
{
  return obj->currentConfiguration();
}
bool Device::currentConfiguration(ConfigurationIn_t configuration)
{
  return obj->currentConfiguration(configuration);
}
void Device::computeForwardKinematics(int flag) {
  return obj->computeForwardKinematics(flag);
}
void Device::computeFramesForwardKinematics() {
  return obj->computeFramesForwardKinematics();
}
void Device::updateGeometryPlacements() {
  return obj->updateGeometryPlacements();
}
// Methods for hpp::manipulation::Device
void Device::setRobotRootPosition(const std::string& robotName,
                          const Transform3s& positionWRTParentJoint)
{
  return obj->setRobotRootPosition(robotName, positionWRTParentJoint);
}

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

bool check(const Device& device)
{
  for (auto const& g: device.obj->grippers.map)
  {
    std::cout << g.first << std::endl;
  }
  return true;
}

std::map<std::string, HandlePtr_t> getDeviceHandles(const Device& device)
{
  return device.obj->handles.map;
}

std::map<std::string, GripperPtr_t> getDeviceGrippers(const Device& device)
{
  return device.obj->grippers.map;
}

std::string getHandleName(const HandlePtr_t& handle)
{
  return handle->name();
}

void setHandleName(const HandlePtr_t& handle, const std::string& name)
{
  handle->name(name);
}

Transform3s getHandleLocalPosition(const HandlePtr_t& handle)
{
  return handle->localPosition();
}

void setHandleLocalPosition(const HandlePtr_t& handle, const Transform3s& se3)
{
  handle->localPosition(se3);
}

std::vector<bool> getHandleMask(const HandlePtr_t& handle)
{
  return handle->mask();
}

void setHandleMask(const HandlePtr_t& handle, const std::vector<bool>& mask)
{
  handle->mask(mask);
}

std::vector<bool> getHandleMaskComp(const HandlePtr_t& handle)
{
  return handle->maskComp();
}

void setHandleMaskComp(const HandlePtr_t& handle, const std::vector<bool>& mask)
{
  handle->maskComp(mask);
}

void exposeHandle() {
  class_<Handle, HandlePtr_t>("Handle", no_init)
    .def("create", &Handle::create)
    .staticmethod("create")
    .add_property("name", &getHandleName, &setHandleName)
    .add_property("localPosition", &getHandleLocalPosition, &setHandleLocalPosition)
    .add_property("mask", &getHandleMask, &setHandleMask)
    .add_property("maskComp", &getHandleMaskComp, &setHandleMaskComp)
    .add_property("clearance",
		  static_cast<value_type (Handle::*)() const >(&Handle::clearance),
		  static_cast<void (Handle::*)(const value_type&)>(&Handle::clearance))
    .def("createGrasp", &Handle::createGrasp)
    .def("createGraspComplement", &Handle::createGraspComplement)
    .def("createGraspAndComplement", &Handle::createGraspAndComplement);
  class_< std::map<std::string, HandlePtr_t> >("HandleMap")
    .def(boost::python::map_indexing_suite< std::map<std::string, HandlePtr_t>, true >());
}
void exposeDevice() {
  void (Device::*cfk)(int) = &Device::computeForwardKinematics;
  // Enable automatic conversion of std::map
  class_<Device>("Device", init <const std::string&> ())
    .def("name", &Device::name, return_value_policy<return_by_value>())
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
    .def("updateGeometryPlacements", &Device::updateGeometryPlacements)
    .def("setRobotRootPosition", &Device::setRobotRootPosition)
    .def("handles", &getDeviceHandles)
    .def("grippers", &getDeviceGrippers);
}
} // namespace manipulation
} // namespace pyhpp
