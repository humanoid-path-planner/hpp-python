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
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <pyhpp/core/steering-method.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct PPWrapper {
  static bool apply(PathProjector* pp, const PathPtr_t path,
                    PathPtr_t& projPath) {
    return pp->apply(path, projPath);
  }

  static tuple py_apply(PathProjector* pp, const PathPtr_t path) {
    PathPtr_t projPath;
    bool res = pp->apply(path, projPath);
    return boost::python::make_tuple(res, projPath);
  }
};

void exposePathProjector() {
  class_<PathProjector, PathProjectorPtr_t, boost::noncopyable>("PathProjector",
                                                                no_init)
      .def("apply", &PPWrapper::apply)
      .def("apply", &PPWrapper::py_apply);

  class_<pathProjector::Progressive, bases<PathProjector>,
         hpp::core::pathProjector::ProgressivePtr_t, boost::noncopyable>(
      "ProgressiveProjector", no_init);

  class_<pathProjector::Dichotomy, bases<PathProjector>,
         hpp::core::pathProjector::DichotomyPtr_t, boost::noncopyable>(
      "DichotomyProjector", no_init);

  class_<pathProjector::Global, bases<PathProjector>,
         hpp::core::pathProjector::GlobalPtr_t, boost::noncopyable>(
      "GlobalProjector", no_init);

  class_<pathProjector::RecursiveHermite, bases<PathProjector>,
         hpp::core::pathProjector::RecursiveHermitePtr_t, boost::noncopyable>(
      "RecursiveHermiteProjector", no_init);

  def(
      "createNoneProjector",
      +[](const DistancePtr_t&, const PyWSteeringMethodPtr_t&,
          const value_type&) -> PathProjectorPtr_t {
        return PathProjectorPtr_t();
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "createProgressiveProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Progressive::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "createDichotomyProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Dichotomy::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "createGlobalProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::Global::create(distance,
                                             steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));

  def(
      "createRecursiveHermiteProjector",
      +[](const DistancePtr_t& distance,
          const PyWSteeringMethodPtr_t& steeringMethodWrapper,
          const value_type& step) {
        return pathProjector::RecursiveHermite::create(
            distance, steeringMethodWrapper->obj, step);
      },
      (arg("distance"), arg("steeringMethod"), arg("step")));
}
}  // namespace core
}  // namespace pyhpp
