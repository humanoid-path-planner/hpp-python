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

// DocNamespace(hpp::manipulation)

namespace pyhpp {
namespace manipulation {
using namespace boost::python;

Device::Device(const std::string& name)
    : pyhpp::pinocchio::Device(hpp::manipulation::Device::create(name)) {}

void Device::setRobotRootPosition(const std::string& robotName,
                                  const Transform3s& positionWRTParentJoint) {
  std::dynamic_pointer_cast<hpp::manipulation::Device>(obj)
      ->setRobotRootPosition(robotName, positionWRTParentJoint);
}

std::map<std::string, HandlePtr_t> Device::handles() {
  return std::dynamic_pointer_cast<hpp::manipulation::Device>(obj)->handles.map;
}

std::map<std::string, GripperPtr_t> Device::grippers() {
  return std::dynamic_pointer_cast<hpp::manipulation::Device>(obj)
      ->grippers.map;
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

boost::python::list Device::contactSurfaceNames() {
  auto manipDevice = std::dynamic_pointer_cast<hpp::manipulation::Device>(obj);
  boost::python::list result;
  for (const auto& entry : manipDevice->jointAndShapes.map) {
    result.append(entry.first);
  }
  return result;
}

boost::python::dict Device::contactSurfaces() {
  auto manipDevice = std::dynamic_pointer_cast<hpp::manipulation::Device>(obj);
  boost::python::dict result;

  for (const auto& entry : manipDevice->jointAndShapes.map) {
    const std::string& name = entry.first;
    const auto& jointAndShapes = entry.second;
    boost::python::list surfacesList;

    for (const auto& jas : jointAndShapes) {
      boost::python::dict surfaceDict;
      // jas.first is JointPtr_t, jas.second is Shape_t (vector<vector3_t>)
      std::string jointName = jas.first ? jas.first->name() : "universe";
      surfaceDict["joint"] = jointName;

      boost::python::list points;
      for (const auto& pt : jas.second) {
        boost::python::list point;
        point.append(pt[0]);
        point.append(pt[1]);
        point.append(pt[2]);
        points.append(point);
      }
      surfaceDict["points"] = points;
      surfacesList.append(surfaceDict);
    }
    result[name] = surfacesList;
  }
  return result;
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
  // DocClass(Handle)
  class_<Handle, HandlePtr_t>("Handle", no_init)
      .def("create", &Handle::create, DocClassMethod(create))
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
      .def("createGrasp", &Handle::createGrasp, DocClassMethod(createGrasp))
      .def("createPreGrasp", &Handle::createPreGrasp,
           DocClassMethod(createPreGrasp))
      .def("createGraspComplement", &Handle::createGraspComplement,
           DocClassMethod(createGraspComplement))
      .def("createGraspAndComplement", &Handle::createGraspAndComplement,
           DocClassMethod(createGraspAndComplement));
  class_<std::map<std::string, HandlePtr_t> >("HandleMap")
      .def(boost::python::map_indexing_suite<std::map<std::string, HandlePtr_t>,
                                             true>());
}
object asPinDevice(object self) {
  PyErr_WarnEx(PyExc_DeprecationWarning,
               "asPinDevice() is deprecated: manipulation.Device already "
               "inherits from pinocchio.Device, use the object directly",
               1);
  return self;
}

void exposeDevice() {
  // DocClass(Device)
  class_<Device, bases<pyhpp::pinocchio::Device>, boost::shared_ptr<Device>,
         boost::noncopyable>("Device", init<const std::string&>())
      .def("setRobotRootPosition", &Device::setRobotRootPosition)
      .def("handles", &Device::handles)
      .def("grippers", &Device::grippers)
      .def("asPinDevice", &asPinDevice)
      .def("getJointNames", &Device::getJointNames)
      .def("getJointConfig", &Device::getJointConfig)
      .def("setJointBounds", &Device::setJointBounds)
      .def("contactSurfaceNames", &Device::contactSurfaceNames,
           "Return list of contact surface names registered on device")
      .def("contactSurfaces", &Device::contactSurfaces,
           "Return dict mapping surface names to list of {joint, points}");
}
}  // namespace manipulation
}  // namespace pyhpp
