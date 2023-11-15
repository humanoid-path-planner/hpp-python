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

#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pyhpp/pinocchio/urdf/fwd.hh>
#include <pyhpp/util.hh>

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

//#include <pinocchio/bindings/python/fwd.hpp>
//#include <pinocchio/bindings/python/multibody/model.hpp>

// Expose pinocchio model again since the template arguments slightly differ
// with the type exposed by pinocchio bindings.
namespace pinocchio
{
namespace python
{
} // namespace python
} // namespace pinocchio


using namespace boost::python;

namespace pyhpp {
namespace pinocchio {
using namespace hpp::pinocchio;

namespace bp=boost::python;
typedef hpp::pinocchio::Model Model;
typedef hpp::pinocchio::ModelPtr_t ModelPtr_t;
using boost::python::class_;
using boost::python::no_init;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload,Model::getFrameId,
                                       1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload,Model::existFrame,
                                       1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,
                                       Model::addJointFrame,1,2)

void exposeModel()
{
  class_<Model, ModelPtr_t, boost::noncopyable>("Model", no_init)
    .add_property("nq", &Model::nq)
    .add_property("nv", &Model::nv)
    .add_property("njoints", &Model::njoints)
    .add_property("nbodies", &Model::nbodies)
    .add_property("nframes", &Model::nframes)
    .add_property("inertias",&Model::inertias)
    .add_property("jointPlacements",&Model::jointPlacements)
    .add_property("joints",&Model::joints)
    .add_property("idx_qs",&Model::idx_qs)
    .add_property("nqs",&Model::nqs)
    .add_property("idx_vs",&Model::idx_vs)
    .add_property("nvs",&Model::nvs)
    .add_property("parents",&Model::parents)
    .add_property("names",&Model::names)
    .def_readwrite("name",&Model::name)
    .def_readwrite("referenceConfigurations", &Model::referenceConfigurations)

    .def_readwrite("rotorInertia",&Model::rotorInertia,
                   "Vector of rotor inertia parameters.")
    .def_readwrite("rotorGearRatio",&Model::rotorGearRatio,
                   "Vector of rotor gear ratio parameters.")
    .def_readwrite("friction",&Model::friction,
                   "Vector of joint friction parameters.")
    .def_readwrite("damping",&Model::damping,
                   "Vector of joint damping parameters.")
    .def_readwrite("effortLimit",&Model::effortLimit,
                   "Joint max effort.")
    .def_readwrite("velocityLimit",&Model::velocityLimit,
                   "Joint max velocity.")
    .def_readwrite("lowerPositionLimit",&Model::lowerPositionLimit,
                   "Limit for joint lower position.")
    .def_readwrite("upperPositionLimit",&Model::upperPositionLimit,
                   "Limit for joint upper position.")
    .def_readwrite("frames",&Model::frames,
                   "Vector of frames contained in the model.")
    .def_readwrite("supports",
                   &Model::supports,
                   "Vector of supports. supports[j] corresponds to the list of joints on the path between\n"
                   "the current *j* to the root of the kinematic tree.")
    .def_readwrite("subtrees",
                   &Model::subtrees,
                   "Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.")
    .def_readwrite("gravity",&Model::gravity,
                   "Motion vector corresponding to the gravity field expressed in the world Frame.")
    .def("appendBodyToJoint",&Model::appendBodyToJoint,
         bp::args("self","joint_id","body_inertia","body_placement"),
         "Appends a body to the joint given by its index. The body is defined by its inertia, its relative placement regarding to the joint and its name.")
    .def("addBodyFrame", &Model::addBodyFrame, bp::args("self","body_name", "parentJoint", "body_placement", "previous_frame"), "add a body to the frame tree")
    .def("getBodyId",&Model::getBodyId, bp::args("self","name"), "Return the index of a frame of type BODY given by its name")
    .def("existBodyName", &Model::existBodyName, bp::args("self","name"), "Check if a frame of type BODY exists, given its name")
    .def("getJointId",&Model::getJointId, bp::args("self","name"), "Return the index of a joint given by its name")
    .def("existJointName", &Model::existJointName, bp::args("self","name"), "Check if a joint given by its name exists")
    .def("getFrameId",&Model::getFrameId,getFrameId_overload(bp::args("self","name","type"),"Returns the index of the frame given by its name and its type. If the frame is not in the frames vector, it returns the current size of the frames vector."))
    .def("existFrame",&Model::existFrame,existFrame_overload(bp::args("self","name","type"),"Returns true if the frame given by its name exists inside the Model with the given type."))
    .def("hasConfigurationLimit",&Model::hasConfigurationLimit, bp::args("self"), "Returns list of boolean if joints have configuration limit.")
    .def("hasConfigurationLimitInTangent",&Model::hasConfigurationLimitInTangent, bp::args("self"), "Returns list of boolean if joints have configuration limit in tangent space  .")
    .def(bp::self == bp::self)
    .def(bp::self != bp::self);
}

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
          PYHPP_DEFINE_METHOD(Device, configSize)
              PYHPP_DEFINE_METHOD(Device, numberDof)

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
