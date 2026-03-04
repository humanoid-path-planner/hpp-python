//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
//

#ifndef PYHPP_PINOCCHIO_DEVICE_HH
#define PYHPP_PINOCCHIO_DEVICE_HH

#include <hpp/manipulation/device.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>

namespace pyhpp {
namespace pinocchio {

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
typedef hpp::pinocchio::DevicePtr_t DevicePtr_t;
typedef hpp::pinocchio::Computation_t Computation_t;
typedef hpp::pinocchio::value_type value_type;

struct Device {
  DevicePtr_t obj;

  Device(const DevicePtr_t& object) : obj(object) {}

  const std::string& name() const { return obj->name(); }
  const LiegroupSpacePtr_t& configSpace() const { return obj->configSpace(); }
  Model& model() { return obj->model(); }
  Data& data() { return obj->data(); }
  GeomData& geomData() { return obj->geomData(); }
  GeomModel& geomModel() { return obj->geomModel(); }
  GeomModel& visualModel() { return obj->visualModel(); }
  size_type configSize() const { return obj->configSize(); }
  size_type numberDof() const { return obj->numberDof(); }
  const Configuration_t& currentConfiguration() const {
    return obj->currentConfiguration();
  }
  bool currentConfiguration(ConfigurationIn_t configuration) {
    return obj->currentConfiguration(configuration);
  }
  void computeForwardKinematics(int flag) {
    obj->computeForwardKinematics(flag);
  }
  void computeFramesForwardKinematics() {
    obj->computeFramesForwardKinematics();
  }
  void updateGeometryPlacements() { obj->updateGeometryPlacements(); }
  void removeJoints(const std::vector<std::string>& jointNames,
                    Configuration_t q) { obj->removeJoints(jointNames, q); }
  hpp::manipulation::DevicePtr_t asManipulationDevice() const {
    auto manipDevice = HPP_DYNAMIC_PTR_CAST(hpp::manipulation::Device, obj);
    if (!manipDevice) {
      throw std::runtime_error("Not a manipulation device²");
    }
    return manipDevice;
  }
};

}  // namespace pinocchio
}  // namespace pyhpp

#endif  // PYHPP_PINOCCHIO_DEVICE_HH
