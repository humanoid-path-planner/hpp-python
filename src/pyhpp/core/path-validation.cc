// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <hpp/core/path-validation.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct PVWrapper {
  static bool validate(PathValidation* pv, const PathPtr_t path, bool reverse,
                       PathPtr_t& validPart,
                       PathValidationReportPtr_t& report) {
    return pv->validate(path, reverse, validPart, report);
  }

  static tuple py_validate(PathValidation* pv, const PathPtr_t path,
                           bool reverse = false) {
    PathPtr_t validPart;
    PathValidationReportPtr_t report;
    bool res = pv->validate(path, reverse, validPart, report);
    return boost::python::make_tuple(res, validPart, report);
  }
};

void exposePathValidation() {
  class_<PathValidation, PathValidationPtr_t, boost::noncopyable>(
      "PathValidation", no_init)
      .def("validate", &PVWrapper::validate)

      .def("validate", &PVWrapper::py_validate);
}
}  // namespace core
}  // namespace pyhpp
