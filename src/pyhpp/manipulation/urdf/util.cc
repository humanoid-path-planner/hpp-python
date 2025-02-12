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

#include <boost/python.hpp>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/srdf/util.hh>
#include <pyhpp/pinocchio/urdf/fwd.hh>

#include <../src/pyhpp/manipulation/device.hh>

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
	       const SE3& bMr)
{
  hpp::pinocchio::urdf::loadModel(robot.obj, baseFrame, prefix, rootType, urdfPath, srdfPath, bMr);
  if (!std::string(srdfPath).empty()) {
    hpp::manipulation::srdf::loadModelFromFile(robot.obj, prefix, srdfPath);
  }
}

void loadModelFromString(const Device& robot, const FrameIndex& baseFrame,
                         const std::string& prefix, const std::string& rootType,
                         const std::string& urdfString,
                         const std::string& srdfString,
                         const SE3& bMr)
{
  hpp::pinocchio::urdf::loadModelFromString(robot.obj, baseFrame, prefix, rootType, urdfString,
					    srdfString, bMr);
  hpp::manipulation::srdf::loadModelFromXML(robot.obj, prefix, srdfString);
}

void exposeUtil() {
  def("loadModel", &loadModel);
  def("loadModelFromString", &loadModelFromString);
}
}  // namespace urdf
}  // namespace manipulation
}  // namespace pyhpp
