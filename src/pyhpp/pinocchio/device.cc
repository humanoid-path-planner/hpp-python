//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <../src/pyhpp/pinocchio/device.hh>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pyhpp/pinocchio/urdf/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::pinocchio)

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

namespace bp = boost::python;
typedef hpp::pinocchio::DevicePtr_t DevicePtr_t;
typedef hpp::pinocchio::GripperPtr_t GripperPtr_t;
typedef hpp::pinocchio::Gripper Gripper;
typedef hpp::pinocchio::FrameIndex FrameIndex;
typedef hpp::pinocchio::SE3 SE3;
typedef hpp::pinocchio::Frame Frame;
typedef hpp::pinocchio::JointPtr_t JointPtr_t;

using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFrameId_overload, Model::getFrameId,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(existFrame_overload, Model::existFrame,
                                       1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addJointFrame_overload,
                                       Model::addJointFrame, 1, 2)

bool Device_currentConfiguration(Device& d, const Configuration_t& c) {
  return d.currentConfiguration(c);
}

Transform3s getObjectPositionInJoint(const GripperPtr_t& gripper) {
  Transform3s res(gripper->objectPositionInJoint());
  return res;
}

static hpp::pinocchio::vector3_t getCenterOfMass(Device& d) {
  try {
    d.computeForwardKinematics(hpp::pinocchio::COM);
    return d.obj->positionCenterOfMass();
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

static void setJointBounds(Device& device, const char* jointName,
                           boost::python::list py_jointBounds) {
  Frame frame = device.obj->getFrameByName(jointName);
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

    joint->lowerBound(i, vMin);
    joint->upperBound(i, vMax);
  }
}

static boost::python::list getJointPosition(Device& device,
                                            const std::string& jointName) {
  try {
    const Model& model(device.model());
    const Data& data(device.data());

    if (!model.existFrame(jointName)) {
      throw std::logic_error("Robot has no frame with name " + jointName);
    }

    FrameIndex joint = model.getFrameId(jointName);
    if (model.frames.size() <= (std::size_t)joint)
      throw std::logic_error("Frame index of joint " + jointName +
                             " out of bounds: " + std::to_string(joint));

    const SE3& M = data.oMf[joint];
    Transform3s::Quaternion q(M.rotation());

    boost::python::list t;
    t.append(M.translation()[0]);
    t.append(M.translation()[1]);
    t.append(M.translation()[2]);
    t.append(q.x());
    t.append(q.y());
    t.append(q.z());
    t.append(q.w());

    return t;
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

static boost::python::dict rankInConfiguration(Device& device) {
  boost::python::dict rank_dict;
  try {
    auto joint_names = device.model().names;
    for (const auto& joint_name : joint_names) {
      Frame frame = device.obj->getFrameByName(joint_name.c_str());
      if (!frame.isFixed()) {
        JointPtr_t joint = frame.joint();
        if (joint) {
          rank_dict[joint_name] = joint->rankInConfiguration();
        }
      }
    }
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
  return rank_dict;
}

static boost::python::list getJointsPosition(
    Device& device, const Configuration_t& dofArray,
    const boost::python::list& jointNames) {
  try {
    device.currentConfiguration(dofArray);
    device.computeForwardKinematics(hpp::pinocchio::JOINT_POSITION);
    device.computeFramesForwardKinematics();

    const Model& model(device.model());
    const Data& data(device.data());
    boost::python::list transforms;
    for (Py_ssize_t i = 0;
         i < static_cast<Py_ssize_t>(boost::python::len(jointNames)); ++i) {
      std::string n = boost::python::extract<std::string>(jointNames[i]);
      if (!model.existFrame(n)) {
        throw std::logic_error("Robot has no frame with name " + n);
      }
      FrameIndex joint = model.getFrameId(n);
      if (model.frames.size() <= (std::size_t)joint)
        throw std::logic_error("Frame index of joint " + n +
                               " out of bounds: " + std::to_string(joint));
      const SE3& M = data.oMf[joint];
      Transform3s::Quaternion q(M.rotation());
      boost::python::list t;
      t.append(M.translation()[0]);
      t.append(M.translation()[1]);
      t.append(M.translation()[2]);
      t.append(q.x());
      t.append(q.y());
      t.append(q.z());
      t.append(q.w());
      transforms.append(t);
    }
    return transforms;
  } catch (const std::exception& exc) {
    throw std::logic_error(exc.what());
  }
}

static std::string getGripperName(const GripperPtr_t& gripper) { return gripper->name(); }

static JointIndex getParentJointId(const GripperPtr_t& gripper) {
  assert(gripper->joint());
  assert(gripper->joint()->robot());
  Model model(gripper->joint()->robot()->model());
  return model.frames[model.getFrameId(gripper->name())].parentJoint;
}

void exposeGripper() {
  // DocClass(Gripper)
  class_<Gripper, GripperPtr_t>("Gripper", no_init)
      .add_property("localPosition", &getObjectPositionInJoint)
      .add_property(
          "clearance",
          static_cast<value_type (Gripper::*)() const>(&Gripper::clearance),
          static_cast<void (Gripper::*)(const value_type&)>(
              &Gripper::clearance))
      .def("name", &getGripperName)
      .def("getParentJointId", &getParentJointId, "Get index of the joint the handle is attached to"
           " in pinocchio Model");
  class_<std::map<std::string, GripperPtr_t> >("GripperMap")
      .def(
          boost::python::map_indexing_suite<std::map<std::string, GripperPtr_t>,
                                            true>());
}

static boost::shared_ptr<Device> createDevice(const std::string& name) {
  return boost::make_shared<Device>(hpp::pinocchio::Device::create(name));
}

struct DeviceWrapperConverter {
  static void* convertible(PyObject* obj_ptr) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Device> extractor(obj);
    return extractor.check() ? obj_ptr : nullptr;
  }

  static void construct_shared_ptr(
      PyObject* obj_ptr,
      boost::python::converter::rvalue_from_python_stage1_data* data) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Device> extractor(obj);

    const Device& wrapper = extractor();
    typedef std::shared_ptr<hpp::pinocchio::Device> PinDevicePtr_t;
    typedef boost::python::converter::rvalue_from_python_storage<PinDevicePtr_t>
        StorageType;

    void* storage = ((StorageType*)data)->storage.bytes;
    new (storage) PinDevicePtr_t(wrapper.obj);
    data->convertible = storage;
  }

  static void construct_const_shared_ptr(
      PyObject* obj_ptr,
      boost::python::converter::rvalue_from_python_stage1_data* data) {
    boost::python::object obj(boost::python::borrowed(obj_ptr));
    boost::python::extract<Device> extractor(obj);

    const Device& wrapper = extractor();
    typedef std::shared_ptr<hpp::pinocchio::Device const> ConstPinDevicePtr_t;
    typedef boost::python::converter::rvalue_from_python_storage<
        ConstPinDevicePtr_t>
        StorageType;

    void* storage = ((StorageType*)data)->storage.bytes;
    new (storage) ConstPinDevicePtr_t(wrapper.obj);
    data->convertible = storage;
  }
};

void register_device_converters() {
  typedef std::shared_ptr<hpp::pinocchio::Device> PinDevicePtr_t;
  typedef std::shared_ptr<hpp::pinocchio::Device const> ConstPinDevicePtr_t;

  boost::python::converter::registry::push_back(
      &DeviceWrapperConverter::convertible,
      &DeviceWrapperConverter::construct_shared_ptr,
      boost::python::type_id<PinDevicePtr_t>());

  boost::python::converter::registry::push_back(
      &DeviceWrapperConverter::convertible,
      &DeviceWrapperConverter::construct_const_shared_ptr,
      boost::python::type_id<ConstPinDevicePtr_t>());
}
void exposeDevice() {
  enum_<Computation_t>("ComputationFlag")
      .value("JOINT_POSITION", hpp::pinocchio::JOINT_POSITION)
      .value("JACOBIAN", hpp::pinocchio::JACOBIAN)
      .value("VELOCITY", hpp::pinocchio::VELOCITY)
      .value("ACCELERATION", hpp::pinocchio::ACCELERATION)
      .value("COM", hpp::pinocchio::COM)
      .value("COMPUTE_ALL", hpp::pinocchio::COMPUTE_ALL);
  void (Device::*cfk)(int) = &Device::computeForwardKinematics;

  // DocClass(Device)
  class_<Device, boost::shared_ptr<Device>, boost::noncopyable>("Device",
                                                                no_init)
      .def("__init__", make_constructor(&createDevice))
      .def("name", &Device::name, return_value_policy<return_by_value>(),
           DocClassMethod(name))
      .def("configSpace", &Device::configSpace,
           return_value_policy<return_by_value>(), DocClassMethod(configSpace))
      .def("model", &Device::model, return_internal_reference<>())
      .def("data", &Device::data, return_internal_reference<>())
      .def("geomData", &Device::geomData, return_internal_reference<>())
      .def("geomModel", &Device::geomModel, return_internal_reference<>())
      .def("configSize", &Device::configSize, DocClassMethod(configSize))
      .def("numberDof", &Device::numberDof, DocClassMethod(numberDof))
      .def("visualModel", &Device::visualModel, return_internal_reference<>())

      .def("currentConfiguration",
           static_cast<const Configuration_t& (Device::*)() const>(
               &Device::currentConfiguration),
           return_value_policy<return_by_value>(),
           DocClassMethod(currentConfiguration))
      .def("currentConfiguration", Device_currentConfiguration)

      .def("computeForwardKinematics", cfk,
           DocClassMethod(computeForwardKinematics))
      .def("computeFramesForwardKinematics",
           &Device::computeFramesForwardKinematics,
           DocClassMethod(computeFramesForwardKinematics))
      .def("updateGeometryPlacements", &Device::updateGeometryPlacements,
           DocClassMethod(updateGeometryPlacements))
      .add_property("rankInConfiguration", &rankInConfiguration)
      .def("setJointBounds", &setJointBounds)
      .def("getCenterOfMass", &getCenterOfMass)
      .def("getJointPosition", &getJointPosition)
      .def("getJointsPosition", &getJointsPosition)
      .def("removeJoints", &Device::removeJoints);
  register_device_converters();
}
}  // namespace pinocchio
}  // namespace pyhpp
