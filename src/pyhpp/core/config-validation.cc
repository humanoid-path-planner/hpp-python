//
// Copyright (c) 2018 - 2023 CNRS
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

#include <boost/python.hpp>
#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

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
      .PYHPP_DEFINE_METHOD2(ConfigValidation, validate,
                            DocClassMethod(validate))
      .def("validate", &CVWrapper::py_validate);

  // DocClass (ConfigValidations)
  class_<ConfigValidations, ConfigValidationsPtr_t, bases<ConfigValidation>,
         boost::noncopyable>("ConfigValidations", no_init)
      .PYHPP_DEFINE_METHOD2(ConfigValidations, add, DocClassMethod(add))
      .PYHPP_DEFINE_METHOD2(ConfigValidations, numberConfigValidations,
                            DocClassMethod(numberConfigValidations))
      .def("clear", &ConfigValidations::clear);
}
}  // namespace core
}  // namespace pyhpp
