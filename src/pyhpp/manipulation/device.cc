//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
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

#include <../src/pyhpp/manipulation/device.hh>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <pyhpp/util.hh>

namespace pyhpp {
namespace manipulation {
using namespace boost::python;

Device::Device(const hpp::manipulation::DevicePtr_t& object) { obj = object; }
Device::Device(const std::string& name)
    : obj(hpp::manipulation::Device::create(name)) {}
// Methods from hpp::pinocchio::Device
const std::string& Device::name() const { return obj->name(); }
const LiegroupSpacePtr_t& Device::configSpace() const {
  return obj->configSpace();
}
Model& Device::model() { return obj->model(); }
Data& Device::data() { return obj->data(); }
GeomData& Device::geomData() { return obj->geomData(); }
GeomModel& Device::geomModel() { return obj->geomModel(); }
GeomModel& Device::visualModel() { return obj->visualModel(); }

size_type Device::configSize() const { return obj->configSize(); }
size_type Device::numberDof() const { return obj->numberDof(); }
const Configuration_t& Device::currentConfiguration() const {
  return obj->currentConfiguration();
}
bool Device::currentConfiguration(ConfigurationIn_t configuration) {
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
                                  const Transform3s& positionWRTParentJoint) {
  return obj->setRobotRootPosition(robotName, positionWRTParentJoint);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload, Model::getFrameId,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload, Model::existFrame,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,
                                       Model::addJointFrame, 1, 2)

PinDevicePtr_t Device::asPinDevice() {
  hpp::pinocchio::DevicePtr_t pinDevice =
      std::dynamic_pointer_cast<hpp::pinocchio::Device>(obj);
  return pinDevice;
}

void Device::setJointBounds(const char* jointName,
                            boost::python::list py_jointBounds) {
  Frame frame = obj->getFrameByName(jointName);
  JointPtr_t joint = frame.joint();
  auto jointBounds = extract_vector<value_type>(py_jointBounds);

  static const value_type inf = std::numeric_limits<value_type>::infinity();

  if (jointBounds.size() % 2 == 1) {
    throw std::logic_error("Expect a vector of even size");
  }

  std::size_t numDofPairs = jointBounds.size() / 2;

  for (std::size_t i = 0; i < numDofPairs; i++) {
    value_type vMin = jointBounds[2 * i];
    value_type vMax = jointBounds[2 * i + 1];

    if (vMin > vMax) {
      vMin = -inf;
      vMax = inf;
    }

    // Set bounds for individual DOF using the old API
    joint->lowerBound(i, vMin);  // Set lower bound for DOF i
    joint->upperBound(i, vMax);  // Set upper bound for DOF i
  }
}
boost::python::list Device::getJointConfig(const char* jointName) {
  try {
    Frame frame = obj->getFrameByName(jointName);
    if (frame.isFixed()) return boost::python::list();
    JointPtr_t joint = frame.joint();
    if (!joint) return boost::python::list();
    hpp::core::vector_t config = obj->currentConfiguration();
    size_type ric = joint->rankInConfiguration();
    size_type dim = joint->configSize();
    auto segment = config.segment(ric, dim);
    std::vector<double> vec(segment.data(), segment.data() + segment.size());
    return to_python_list(vec);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

boost::python::list Device::getJointNames() {
  try {
    const Model& model = obj->model();
    return to_python_list(model.names);
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

bool Device_currentConfiguration(Device& d, const Configuration_t& c) {
  return d.currentConfiguration(c);
}

Transform3s getObjectPositionInJoint(const GripperPtr_t& gripper) {
  Transform3s res(gripper->objectPositionInJoint());
  return res;
}

bool check(const Device& device) {
  for (auto const& g : device.obj->grippers.map) {
    std::cout << g.first << std::endl;
  }
  return true;
}

std::map<std::string, HandlePtr_t> getDeviceHandles(const Device& device) {
  return device.obj->handles.map;
}

std::map<std::string, GripperPtr_t> getDeviceGrippers(const Device& device) {
  return device.obj->grippers.map;
}

std::string getHandleName(const HandlePtr_t& handle) { return handle->name(); }

void setHandleName(const HandlePtr_t& handle, const std::string& name) {
  handle->name(name);
}

Transform3s getHandleLocalPosition(const HandlePtr_t& handle) {
  return handle->localPosition();
}

void setHandleLocalPosition(const HandlePtr_t& handle, const Transform3s& se3) {
  handle->localPosition(se3);
}

std::vector<bool> getHandleMask(const HandlePtr_t& handle) {
  return handle->mask();
}

void setHandleMask(const HandlePtr_t& handle, const std::vector<bool>& mask) {
  handle->mask(mask);
}

std::vector<bool> getHandleMaskComp(const HandlePtr_t& handle) {
  return handle->maskComp();
}

void setHandleMaskComp(const HandlePtr_t& handle,
                       const std::vector<bool>& mask) {
  handle->maskComp(mask);
}

void exposeHandle() {
  class_<Handle, HandlePtr_t>("Handle", no_init)
      .def("create", &Handle::create)
      .staticmethod("create")
      .add_property("name", &getHandleName, &setHandleName)
      .add_property("localPosition", &getHandleLocalPosition,
                    &setHandleLocalPosition)
      .add_property("mask", &getHandleMask, &setHandleMask)
      .add_property("maskComp", &getHandleMaskComp, &setHandleMaskComp)
      .add_property(
          "clearance",
          static_cast<value_type (Handle::*)() const>(&Handle::clearance),
          static_cast<void (Handle::*)(const value_type&)>(&Handle::clearance))
      .def("createGrasp", &Handle::createGrasp)
      .def("createPreGrasp", &Handle::createPreGrasp)
      .def("createGraspComplement", &Handle::createGraspComplement)
      .def("createGraspAndComplement", &Handle::createGraspAndComplement);
  class_<std::map<std::string, HandlePtr_t> >("HandleMap")
      .def(boost::python::map_indexing_suite<std::map<std::string, HandlePtr_t>,
                                             true>());
}
void exposeDevice() {
  void (Device::*cfk)(int) = &Device::computeForwardKinematics;
  // Enable automatic conversion of std::map
  class_<Device>("Device", init<const std::string&>())
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
      .def("visualModel",
           static_cast<GeomModel& (Device::*)()>(&Device::visualModel),
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
      .def("grippers", &getDeviceGrippers)
      .def("asPinDevice", &Device::asPinDevice)
      .def("getJointNames", &Device::getJointNames)
      .def("getJointConfig", &Device::getJointConfig)
      .def("setJointBounds", &Device::setJointBounds);
}
}  // namespace manipulation
}  // namespace pyhpp
