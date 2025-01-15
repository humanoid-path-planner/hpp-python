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
#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device.hh>
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
using namespace hpp::pinocchio;

namespace bp = boost::python;
typedef hpp::pinocchio::Model Model;
typedef hpp::pinocchio::ModelPtr_t ModelPtr_t;
using boost::python::class_;
using boost::python::no_init;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload, Model::getFrameId,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload, Model::existFrame,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,
                                       Model::addJointFrame, 1, 2)

bool Device_currentConfiguration(Device& d, const Configuration_t& c) {
  return d.currentConfiguration(c);
}

void exposeDevice() {
  enum_<Computation_t>("ComputationFlag")
      .value("JOINT_POSITION", JOINT_POSITION)
      .value("JACOBIAN", JACOBIAN)
      .value("VELOCITY", VELOCITY)
      .value("ACCELERATION", ACCELERATION)
      .value("COM", COM)
      .value("COMPUTE_ALL", COMPUTE_ALL);
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
