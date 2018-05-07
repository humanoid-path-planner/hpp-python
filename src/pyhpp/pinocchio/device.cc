//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
//
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

#include <pyhpp/pinocchio/urdf/fwd.hh>

#include <boost/python.hpp>

#include <eigenpy/eigenpy.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/device.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace pinocchio {
    using namespace hpp::pinocchio;

    bool Device_currentConfiguration (Device& d, const Configuration_t& c)
    {
      return d.currentConfiguration(c);
    }

    void exposeDevice()
    {
      enum_ <Device::Computation_t> ("ComputationFlag")
        .value ("JOINT_POSITION", Device::JOINT_POSITION)
        .value ("JACOBIAN"      , Device::JACOBIAN      )
        .value ("VELOCITY"      , Device::VELOCITY      )
        .value ("ACCELERATION"  , Device::ACCELERATION  )
        .value ("COM"           , Device::COM           )
        .value ("ALL"           , Device::ALL           )
        ;
      class_<Device, DevicePtr_t, boost::noncopyable> ("Device", no_init)
        .def ("name",   &Device::name, return_value_policy<return_by_value>())
        .def ("create", &Device::create)
        .staticmethod("create")

        .def ("model",     static_cast<Model    & (Device::*) ()> (&Device::model    ), return_internal_reference<>())
        .def ("data",      static_cast<Data     & (Device::*) ()> (&Device::data     ), return_internal_reference<>())
        .def ("geomData",  static_cast<GeomData & (Device::*) ()> (&Device::geomData ), return_internal_reference<>())
        .def ("geomModel", static_cast<GeomModel& (Device::*) ()> (&Device::geomModel), return_internal_reference<>())
        PYHPP_DEFINE_METHOD (Device, configSize)
        PYHPP_DEFINE_METHOD (Device, numberDof)

        .def ("currentConfiguration", static_cast<const Configuration_t& (Device::*) () const> (&Device::currentConfiguration), return_value_policy<return_by_value>())
        .def ("currentConfiguration", Device_currentConfiguration)

        .add_property ("computationFlag",
            &Device::computationFlag,
            &Device::controlComputation)
        .def ("computeForwardKinematics",       &Device::computeForwardKinematics)
        .def ("computeFramesForwardKinematics", &Device::computeFramesForwardKinematics)
        .def ("updateGeometryPlacements",       &Device::updateGeometryPlacements)
        ;
    }
  }
}
