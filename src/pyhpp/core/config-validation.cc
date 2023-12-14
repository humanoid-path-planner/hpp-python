//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

#include <boost/python.hpp>

using namespace boost::python;

// DocNamespace(hpp::core)

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct CVWrapper {
  static bool validate(ConfigValidation* cv, const Configuration_t& config,
                       ValidationReportPtr_t& report) {
    return cv->validate(config, report);
  }

  static tuple py_validate(ConfigValidation* cv,
                           const Configuration_t& config) {
    ValidationReportPtr_t report;
    bool res = cv->validate(config, report);
    return boost::python::make_tuple(res, report);
  }
};

void exposeConfigValidation() {
  // DocClass (ConfigValidation)
  class_<ConfigValidation, ConfigValidationPtr_t, boost::noncopyable>(
      "ConfigValidation", no_init)
    .PYHPP_DEFINE_METHOD2(ConfigValidation, validate, DocClassMethod(validate))
    .def("validate", &CVWrapper::py_validate);

  // DocClass (ConfigValidations)
  class_<ConfigValidations, ConfigValidationsPtr_t, bases<ConfigValidation>,
         boost::noncopyable>("ConfigValidations", no_init)
      .PYHPP_DEFINE_METHOD2(ConfigValidations, add, DocClassMethod(add))
      .PYHPP_DEFINE_METHOD2(ConfigValidations, numberConfigValidations,
          DocClassMethod(numberConfigValidations))
      .PYHPP_DEFINE_METHOD2(ConfigValidations, clear,
          DocClassMethod(clear));
}
}  // namespace core
}  // namespace pyhpp
