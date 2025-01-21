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
#include <pinocchio/spatial/se3.hpp>
#include <hpp/constraints/implicit.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>

namespace pyhpp {
namespace manipulation {
using namespace boost::python;
using namespace hpp::manipulation;

std::map<std::string, HandlePtr_t> getDeviceHandles(const DevicePtr_t& device)
{
  return device->handles.map;
}

std::map<std::string, GripperPtr_t> getDeviceGrippers(const DevicePtr_t& device)
{
  return device->grippers.map;
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
  // Enable automatic conversion of std::map
  class_<Device, DevicePtr_t, boost::noncopyable, bases<hpp::pinocchio::Device> >("Device", no_init)
    .def("create", &Device::create)
    .staticmethod("create")
    .def("setRobotRootPosition", &Device::setRobotRootPosition)
    .def("handles", &getDeviceHandles)
    .def("grippers", &getDeviceGrippers);
}
} // namespace manipulation
} // namespace pyhpp
