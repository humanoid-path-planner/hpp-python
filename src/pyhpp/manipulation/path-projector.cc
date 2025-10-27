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

#include <../src/pyhpp/manipulation/steering-method.hh>
#include <boost/python.hpp>
#include <hpp/core/distance.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <pyhpp/core/steering-method.hh>
#include <pyhpp/manipulation/fwd.hh>

using namespace boost::python;

namespace pyhpp {
namespace manipulation {
using namespace hpp::core;
typedef pyhpp::core::PyWSteeringMethodPtr_t PyWSteeringMethodPtr_t;

using namespace boost::python;
void exposePathProjector() {
  def(
      "NoneProjector",
      +[]() -> PathProjectorPtr_t { return PathProjectorPtr_t(); });

  def(
      "ProgressiveProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Progressive::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
  def(
      "ProgressiveProjector",
      +[](const DistancePtr_t& distance,
          const PyWGraphSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Progressive::create(
            distance, steeringMethodWrapper->obj->innerSteeringMethod(), step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "DichotomyProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Dichotomy::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
  def(
      "DichotomyProjector",
      +[](const DistancePtr_t& distance,
          const PyWGraphSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Dichotomy::create(
            distance, steeringMethodWrapper->obj->innerSteeringMethod(), step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "GlobalProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Global::create(distance,
                                             steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
  def(
      "GlobalProjector",
      +[](const DistancePtr_t& distance,
          const PyWGraphSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Global::create(
            distance, steeringMethodWrapper->obj->innerSteeringMethod(), step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "RecursiveHermiteProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::RecursiveHermite::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
  def(
      "RecursiveHermiteProjector",
      +[](const DistancePtr_t& distance,
          const PyWGraphSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::RecursiveHermite::create(
            distance, steeringMethodWrapper->obj->innerSteeringMethod(), step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
}
}  // namespace manipulation
}  // namespace pyhpp
