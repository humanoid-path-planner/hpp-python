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

#ifndef PYHPP_MANIPULATION_DEVICE_HH
#define PYHPP_MANIPULATION_DEVICE_HH

#include <../src/pyhpp/pinocchio/device.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pyhpp/manipulation/fwd.hh>

namespace pyhpp {
namespace manipulation {

typedef hpp::pinocchio::Model Model;
typedef hpp::pinocchio::Data Data;
typedef hpp::pinocchio::ModelPtr_t ModelPtr_t;
typedef hpp::pinocchio::GeomData GeomData;
typedef hpp::pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;
typedef hpp::pinocchio::GeomModel GeomModel;
typedef hpp::pinocchio::Configuration_t Configuration_t;
typedef hpp::pinocchio::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::pinocchio::vector_t vector_t;
typedef hpp::pinocchio::vector3_t vector3_t;
typedef hpp::pinocchio::size_type size_type;
typedef hpp::pinocchio::value_type value_type;
typedef hpp::pinocchio::Transform3s Transform3s;
typedef hpp::pinocchio::GripperPtr_t GripperPtr_t;
typedef hpp::pinocchio::Gripper Gripper;
typedef hpp::pinocchio::Computation_t Computation_t;
typedef hpp::manipulation::HandlePtr_t HandlePtr_t;
typedef hpp::manipulation::Handle Handle;
typedef hpp::manipulation::DevicePtr_t DevicePtr_t;
typedef hpp::pinocchio::DevicePtr_t PinDevicePtr_t;
typedef hpp::pinocchio::Frame Frame;
typedef hpp::pinocchio::FrameIndex FrameIndex;
typedef hpp::pinocchio::Joint Joint;
typedef hpp::pinocchio::JointIndex JointIndex;
typedef hpp::pinocchio::JointPtr_t JointPtr_t;

// Wrapper class to hpp::manipulation::Device
//
// Boost python does not correctly handle weak pointers. To overcome this
// limitation, we create a wrapper class and bind this class with boost python
// instead of the original class.
struct Device : public pyhpp::pinocchio::Device {
  Device(const std::string& name);

  void setRobotRootPosition(const std::string& robotName,
                            const Transform3s& positionWRTParentJoint);
  std::map<std::string, HandlePtr_t> handles();
  std::map<std::string, GripperPtr_t> grippers();
  boost::python::list getJointConfig(const char* jointName);
  boost::python::list getJointNames();
  void setJointBounds(const char* jointName, boost::python::list jointBounds);
  boost::python::list contactSurfaceNames();
  boost::python::dict contactSurfaces();
  void addHandle(const std::string& linkName, const std::string& handleName,
                 const Transform3s& pose, value_type clearance,
                 const std::vector<bool>& mask);
  void addGripper(const std::string& linkName, const std::string& gripperName,
                  const Transform3s& pose, value_type clearance);
};  // struct Device
}  // namespace manipulation
}  // namespace pyhpp
#endif  // PYHPP_MANIPULATION_DEVICE_HH
