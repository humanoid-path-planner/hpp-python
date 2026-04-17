//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
//
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
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimization/rs-time-parameterization.hh>
#include <hpp/core/path-optimization/simple-shortcut.hh>
#include <hpp/core/path-optimization/simple-time-parameterization.hh>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <pyhpp/core/fwd.hh>

// DocNamespace(hpp::core)

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

template <int Order>
void exposeSplineGradientBased(const char* name) {
  typedef pathOptimization::SplineGradientBased<path::BernsteinBasis, Order>
      SGB_t;
  class_<SGB_t, std::shared_ptr<SGB_t>, bases<PathOptimizer>,
         boost::noncopyable>(name, no_init)
      .def("__init__", make_constructor(&SGB_t::create))
      .def_readwrite("alphaInit", &SGB_t::alphaInit)
      .def_readwrite("alwaysStopAtFirst", &SGB_t::alwaysStopAtFirst)
      .def_readwrite("costOrder", &SGB_t::costOrder)
      .def_readwrite("usePathLengthAsWeights", &SGB_t::usePathLengthAsWeights)
      .def_readwrite("reorderIntervals", &SGB_t::reorderIntervals)
      .def_readwrite("linearizeAtEachStep", &SGB_t::linearizeAtEachStep)
      .def_readwrite("checkJointBound", &SGB_t::checkJointBound)
      .def_readwrite("returnOptimum", &SGB_t::returnOptimum)
      .def_readwrite("costThreshold", &SGB_t::costThreshold)
      .def_readwrite("guessThreshold", &SGB_t::guessThreshold)
      .def_readwrite("QPAccuracy", &SGB_t::QPAccuracy);
}

void exposePathOptimizer() {
  // DocClass(PathOptimizer)
  class_<PathOptimizer, PathOptimizerPtr_t, boost::noncopyable>("PathOptimizer",
                                                                no_init)
      .def("problem", &PathOptimizer::problem, DocClassMethod(problem))
      .def("optimize", &PathOptimizer::optimize, DocClassMethod(optimize))
      .def("interrupt", &PathOptimizer::interrupt, DocClassMethod(interrupt))
      .def("maxIterations", &PathOptimizer::maxIterations,
           DocClassMethod(maxIterations))
      .def("timeOut", &PathOptimizer::timeOut, DocClassMethod(timeOut));

  class_<pathOptimization::RandomShortcut,
         std::shared_ptr<pathOptimization::RandomShortcut>,
         bases<PathOptimizer>, boost::noncopyable>("RandomShortcut", no_init)
      .def("__init__",
           make_constructor(&pathOptimization::RandomShortcut::create));

  class_<pathOptimization::SimpleShortcut,
         std::shared_ptr<pathOptimization::SimpleShortcut>,
         bases<PathOptimizer>, boost::noncopyable>("SimpleShortcut", no_init)
      .def("__init__",
           make_constructor(&pathOptimization::SimpleShortcut::create));

  class_<pathOptimization::PartialShortcut,
         std::shared_ptr<pathOptimization::PartialShortcut>,
         bases<PathOptimizer>, boost::noncopyable>("PartialShortcut", no_init)
      .def("__init__",
           make_constructor(&pathOptimization::PartialShortcut::create));

  class_<pathOptimization::SimpleTimeParameterization,
         std::shared_ptr<pathOptimization::SimpleTimeParameterization>,
         bases<PathOptimizer>, boost::noncopyable>("SimpleTimeParameterization",
                                                   no_init)
      .def("__init__",
           make_constructor(
               &pathOptimization::SimpleTimeParameterization::create))
      .def_readwrite("safety",
                     &pathOptimization::SimpleTimeParameterization::safety)
      .def_readwrite("order",
                     &pathOptimization::SimpleTimeParameterization::order)
      .def_readwrite(
          "maxAcceleration",
          &pathOptimization::SimpleTimeParameterization::maxAcceleration);

  class_<pathOptimization::RSTimeParameterization,
         std::shared_ptr<pathOptimization::RSTimeParameterization>,
         bases<PathOptimizer>, boost::noncopyable>("RSTimeParameterization",
                                                   no_init)
      .def("__init__",
           make_constructor(&pathOptimization::RSTimeParameterization::create));

  exposeSplineGradientBased<1>("SplineGradientBased_bezier1");
  exposeSplineGradientBased<3>("SplineGradientBased_bezier3");
  exposeSplineGradientBased<5>("SplineGradientBased_bezier5");
}
}  // namespace core
}  // namespace pyhpp
