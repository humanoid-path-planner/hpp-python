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

#include <boost/python.hpp>
#include <hpp/pinocchio/urdf/util.hh>
#include <pyhpp/pinocchio/urdf/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace pinocchio {
namespace urdf {
using hpp::pinocchio::DevicePtr_t;
using namespace hpp::pinocchio::urdf;

void exposeUtil() {
  def("loadRobotModel",
      static_cast<void (*)(
          const DevicePtr_t& robot, const std::string& rootJointType,
          const std::string& package, const std::string& modelName,
          const std::string& urdfSuffix, const std::string& srdfSuffix)>(
          &loadRobotModel));
}
}  // namespace urdf
}  // namespace pinocchio
}  // namespace pyhpp
