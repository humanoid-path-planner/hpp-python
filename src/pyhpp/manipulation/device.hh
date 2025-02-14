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

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>

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
typedef hpp::pinocchio::size_type size_type;
typedef hpp::pinocchio::value_type value_type;
typedef hpp::pinocchio::Transform3s Transform3s;
typedef hpp::pinocchio::GripperPtr_t GripperPtr_t;
typedef hpp::pinocchio::Gripper Gripper;
typedef hpp::pinocchio::Computation_t Computation_t;
typedef hpp::manipulation::HandlePtr_t HandlePtr_t;
typedef hpp::manipulation::Handle Handle;

// Wrapper class to hpp::manipulation::Device
//
// Boost python does not correctly handle weak pointers. To overcome this limitation,
// we create a wrapper class and bind this class with boost python instead of the original class.
struct Device {
  hpp::manipulation::DevicePtr_t obj;
  Device(const hpp::manipulation::DevicePtr_t& object);
  Device(const std::string& name);
  // Methods from hpp::pinocchio::Device
  const std::string& name() const;
  const LiegroupSpacePtr_t& configSpace() const;
  Model& model();
  Data& data();
  GeomData& geomData();
  GeomModel& geomModel();
  size_type configSize() const;
  size_type numberDof() const;
  const Configuration_t& currentConfiguration() const;
  bool currentConfiguration(ConfigurationIn_t configuration);
  void computeForwardKinematics(int flag);
  void computeFramesForwardKinematics();
  void updateGeometryPlacements();
  // Methods for hpp::manipulation::Device
  void setRobotRootPosition(const std::string& robotName,
                            const Transform3s& positionWRTParentJoint);
}; // struct Device
} // namespace manipulation
} // namespace pyhpp
