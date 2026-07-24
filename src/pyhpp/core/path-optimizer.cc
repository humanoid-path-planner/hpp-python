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
#include <hpp/core/path-optimization/trapezoidal-time-parameterization.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <pyhpp/core/fwd.hh>

// DocNamespace(hpp::core)

namespace {
const char* DOC_TTP_MAXVEL =
    "Maximum velocity for each output degree of freedom.";
const char* DOC_TTP_MAXACC =
    "Maximum acceleration for each output degree of freedom.";
const char* DOC_TTP_MINDUR =
    "Minimum duration assigned to each non-empty subpath.";
const char* DOC_STP_SAFETY = "A scaling factor for the velocity bounds.";
const char* DOC_STP_ORDER = "The desired continuity order (0, 1, or 2).";
const char* DOC_STP_MAXACC =
    "The maximum acceleration for each degree of freedom. Not considered if "
    "negative.";
const char* DOC_SGB_ALPHAINIT =
    "In [0,1]. Initial value when interpolating between non-colliding current "
    "solution and the optimal colliding trajectory.";
const char* DOC_SGB_ALWAYSSTOPFIRST =
    "If true, consider only one (not all) collision constraint per iteration.";
const char* DOC_SGB_COSTORDER =
    "Order of the derivative used for the optimized cost function (most likely "
    "1, 2, or 3).";
const char* DOC_SGB_USEPATHLENGTH =
    "If true, the initial path length is used to weight the splines.";
const char* DOC_SGB_REORDERINTERVALS =
    "If true, intervals in collision are checked first at the next iteration.";
const char* DOC_SGB_LINEARIZE =
    "If true, collision constraint will be re-linearized at each iteration.";
const char* DOC_SGB_CHECKJOINTBOUND = "If true, joint bounds are enforced.";
const char* DOC_SGB_RETURNOPTIMUM =
    "If true, returns the optimum regardless of collision (for debugging).";
const char* DOC_SGB_COSTTHRESHOLD =
    "Stop optimizing if the cost improves less than this threshold between two "
    "iterations.";
const char* DOC_SGB_GUESSTHRESHOLD =
    "Threshold to detect rows of zeros in the Jacobian (passive DoF). Negative "
    "disables the check.";
const char* DOC_SGB_QPACCURACY =
    "Accuracy of the QP solver (only used by proxqp).";
}  // namespace

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
      .def_readwrite("alphaInit", &SGB_t::alphaInit, DOC_SGB_ALPHAINIT)
      .def_readwrite("alwaysStopAtFirst", &SGB_t::alwaysStopAtFirst,
                     DOC_SGB_ALWAYSSTOPFIRST)
      .def_readwrite("costOrder", &SGB_t::costOrder, DOC_SGB_COSTORDER)
      .def_readwrite("usePathLengthAsWeights", &SGB_t::usePathLengthAsWeights,
                     DOC_SGB_USEPATHLENGTH)
      .def_readwrite("reorderIntervals", &SGB_t::reorderIntervals,
                     DOC_SGB_REORDERINTERVALS)
      .def_readwrite("linearizeAtEachStep", &SGB_t::linearizeAtEachStep,
                     DOC_SGB_LINEARIZE)
      .def_readwrite("checkJointBound", &SGB_t::checkJointBound,
                     DOC_SGB_CHECKJOINTBOUND)
      .def_readwrite("returnOptimum", &SGB_t::returnOptimum,
                     DOC_SGB_RETURNOPTIMUM)
      .def_readwrite("costThreshold", &SGB_t::costThreshold,
                     DOC_SGB_COSTTHRESHOLD)
      .def_readwrite("guessThreshold", &SGB_t::guessThreshold,
                     DOC_SGB_GUESSTHRESHOLD)
      .def_readwrite("QPAccuracy", &SGB_t::QPAccuracy, DOC_SGB_QPACCURACY);
}

void exposePathOptimizer() {
  // DocClass(PathOptimizer)
  class_<PathOptimizer, PathOptimizerPtr_t, boost::noncopyable>(
      "PathOptimizer", DocClassDoc(), no_init)
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
                     &pathOptimization::SimpleTimeParameterization::safety,
                     DOC_STP_SAFETY)
      .def_readwrite("order",
                     &pathOptimization::SimpleTimeParameterization::order,
                     DOC_STP_ORDER)
      .def_readwrite(
          "maxAcceleration",
          &pathOptimization::SimpleTimeParameterization::maxAcceleration,
          DOC_STP_MAXACC);

  class_<pathOptimization::RSTimeParameterization,
         std::shared_ptr<pathOptimization::RSTimeParameterization>,
         bases<PathOptimizer>, boost::noncopyable>("RSTimeParameterization",
                                                   no_init)
      .def("__init__",
           make_constructor(&pathOptimization::RSTimeParameterization::create));

  typedef pathOptimization::TrapezoidalTimeParameterization TTP_t;
  class_<TTP_t, std::shared_ptr<TTP_t>, bases<PathOptimizer>,
         boost::noncopyable>("TrapezoidalTimeParameterization", no_init)
      .def("__init__",
           make_constructor(
               &pathOptimization::TrapezoidalTimeParameterization::create))
      .add_property(
          "maxVelocity",
          static_cast<value_type (TTP_t::*)() const>(&TTP_t::maxVelocity),
          static_cast<void (TTP_t::*)(const value_type&)>(&TTP_t::maxVelocity),
          DOC_TTP_MAXVEL)
      .add_property(
          "maxAcceleration",
          static_cast<value_type (TTP_t::*)() const>(&TTP_t::maxAcceleration),
          static_cast<void (TTP_t::*)(const value_type&)>(
              &TTP_t::maxAcceleration),
          DOC_TTP_MAXACC)
      .add_property(
          "minimumDuration",
          static_cast<value_type (TTP_t::*)() const>(&TTP_t::minimumDuration),
          static_cast<void (TTP_t::*)(const value_type&)>(
              &TTP_t::minimumDuration),
          DOC_TTP_MINDUR);

  exposeSplineGradientBased<1>("SplineGradientBased_bezier1");
  exposeSplineGradientBased<3>("SplineGradientBased_bezier3");
  exposeSplineGradientBased<5>("SplineGradientBased_bezier5");
}
}  // namespace core
}  // namespace pyhpp
