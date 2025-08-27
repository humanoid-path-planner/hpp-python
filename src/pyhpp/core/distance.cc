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

#include <boost/python.hpp>
#include <hpp/core/distance.hh>
#include <hpp/core/weighed-distance.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {

using namespace hpp::core;

struct DistanceWrapper {
  static DistancePtr_t AsDistancePtr_t(WeighedDistance& dist) {
    return dist.clone();
  }
  static value_type compute(WeighedDistance* dist, ConfigurationIn_t q1,
			    ConfigurationIn_t q2) {
    return dist->compute(q1, q2);
  }
  static vector_t getWeights(WeighedDistance* dist) {
    return dist->weights();
  }
  static void setWeights(WeighedDistance* dist, const vector_t& weights) {
    return dist->weights(weights);
  }
};

void exposeDistance() {
  class_<Distance, DistancePtr_t, boost::noncopyable>("Distance", no_init)
      .def("compute", &DistanceWrapper::compute);
  class_<WeighedDistance, bases<Distance>, WeighedDistancePtr_t, boost::noncopyable>(
      "WeighedDistance", no_init)
      .def("create", &WeighedDistance::create)
      .staticmethod("create")
      .def("asDistancePtr_t", &DistanceWrapper::AsDistancePtr_t)
      .def("getWeights", &DistanceWrapper::getWeights)
      .def("setWeights", &DistanceWrapper::setWeights)
  ;
}
}  // namespace core
}  // namespace pyhpp
