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
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/srdf/util.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pyhpp/pinocchio/urdf/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace manipulation {
namespace urdf {
using hpp::manipulation::DevicePtr_t;
using ::pinocchio::FrameIndex;
using ::pinocchio::SE3;

// Redefine loadModel in order to parse the srdf file with hpp-manipulation-urdf
void loadModel(const Device& robot, const FrameIndex& baseFrame,
               const std::string& prefix, const std::string& rootType,
               const std::string& urdfPath, const std::string& srdfPath,
               const SE3& bMr) {
  hpp::pinocchio::urdf::loadModel(robot.obj, baseFrame, prefix, rootType,
                                  urdfPath, srdfPath, bMr);
  if (!std::string(srdfPath).empty()) {
    hpp::manipulation::srdf::loadModelFromFile(robot.asManipulationDevice(),
                                               prefix, srdfPath);
  }
}

void loadModelFromString(const Device& robot, const FrameIndex& baseFrame,
                         const std::string& prefix, const std::string& rootType,
                         const std::string& urdfString,
                         const std::string& srdfString, const SE3& bMr) {
  hpp::pinocchio::urdf::loadModelFromString(
      robot.obj, baseFrame, prefix, rootType, urdfString, srdfString, bMr);
  hpp::manipulation::srdf::loadModelFromXML(robot.asManipulationDevice(),
                                            prefix, srdfString);
}

void exposeUtil() {
  def("loadModel", &loadModel);
  def("loadModelFromString", &loadModelFromString);
}
}  // namespace urdf
}  // namespace manipulation
}  // namespace pyhpp
