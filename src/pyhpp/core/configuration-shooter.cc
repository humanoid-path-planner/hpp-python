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
#include <hpp/core/configuration-shooter/gaussian.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::core)

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct CSWrapper {
  static vector_t shoot(const ConfigurationShooter* cs) { return cs->shoot(); }
};

void exposeConfigurationShooter() {
  // DocClass(ConfigurationShooter)
  class_<ConfigurationShooter, ConfigurationShooterPtr_t, boost::noncopyable>(
      "ConfigurationShooter", DocClassDoc(), no_init)
      .def("shoot", &CSWrapper::shoot, DocClassMethod(shoot));
  class_<configurationShooter::Gaussian, configurationShooter::GaussianPtr_t,
	 bases<ConfigurationShooter>, boost::noncopyable>(
	"GaussianShooter", DocClassDoc(), no_init)
    .def("__init__", make_constructor(&configurationShooter::Gaussian::create))
    .def("center", static_cast<void (configurationShooter::Gaussian::*)(ConfigurationIn_t)>(
	&configurationShooter::Gaussian::center))
    .def("center", static_cast<const Configuration_t& (configurationShooter::Gaussian::*)() const>(
        &configurationShooter::Gaussian::center), return_value_policy<return_by_value>())
    .def("sigma", static_cast<void (configurationShooter::Gaussian::*)(const value_type&)>(
        &configurationShooter::Gaussian::sigma))
    .def("sigmas", static_cast<void (configurationShooter::Gaussian::*)(vectorIn_t)>(
        &configurationShooter::Gaussian::sigmas))
    .def("sigmas", static_cast<const vector_t& (configurationShooter::Gaussian::*)() const>(
        &configurationShooter::Gaussian::sigmas), return_value_policy<return_by_value>());
}
}  // namespace core
}  // namespace pyhpp
