//
// Copyright (c) 2026, CNRS
// Authors: Florent Lamiraux
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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>

#include <pyhpp/manipulation/steering-method/cartesian.hh>

// DocNamespace(hpp::manipulation::steeringMethod)

namespace pyhpp {
namespace manipulation {
namespace steeringMethod {
  using boost::python::class_;
  
  Cartesian::Cartesian(const pyhpp::core::Problem& problem) :
    obj(hpp::manipulation::steeringMethod::Cartesian::create(problem.obj)) {
  }

  void Cartesian::setMaxIterations(size_type iterations) {
    obj->maxIterations(iterations);
  }
  size_type Cartesian::getMaxIterations() const {
    return obj->maxIterations();
  }
  void Cartesian::setErrorThreshold(value_type threshold) {
    obj->errorThreshold(threshold);
  }
  value_type Cartesian::getErrorThreshold() const {
    return obj->errorThreshold();
  }
  void Cartesian::setTrajectoryConstraint(const ImplicitPtr_t& ic) {
    obj->trajectoryConstraint(ic);
  }
  ImplicitPtr_t Cartesian::getTrajectoryConstraint() {
    return obj->trajectoryConstraint();
  }
  void Cartesian::setRightHandSide1(const PathPtr_t& rhs, bool se3Output) {
    obj->rightHandSide(rhs, se3Output);
  }
  void Cartesian::setRightHandSide2(const DifferentiableFunctionPtr_t& rhs,
				    const interval_t& timeRange) {
    return obj->rightHandSide(rhs, timeRange);
  }
  DifferentiableFunctionPtr_t Cartesian::getRightHandSide() const {
    return obj->rightHandSide();
  }
  interval_t Cartesian::getTimeRange() const {
    return obj->timeRange();
  }
  size_type Cartesian::getNDiscreteSteps() const {
    return obj->nDiscreteSteps();
  }
  void Cartesian::setNDiscreteSteps(size_type n) {
    obj->nDiscreteSteps(n);
  }
  boost::python::tuple Cartesian::planPath(const Configuration_t& q_init) {
    PathPtr_t result;
    bool success;
    success = obj->planPath(q_init, result);
    return boost::python::make_tuple(success, result);
  }
  
  void exposeCartesian() {
    // DocClass(Cartesian)
    boost::python::class_<Cartesian>("Cartesian",
				     boost::python::init<const pyhpp::core::Problem&>())
      .add_property("maxIterations", &Cartesian::getMaxIterations, &Cartesian::setMaxIterations,
		    "Maximal number of iterations of numerical solver.")
      .add_property("errorThreshold", &Cartesian::getErrorThreshold, &Cartesian::setErrorThreshold,
		    "Error threshold of numerical solver.")
      .add_property("trajectoryConstraint", &Cartesian::getTrajectoryConstraint,
		    &Cartesian::setTrajectoryConstraint,
		    "Constraint with a time-varying right hand side.")
      .add_property("nDiscreteSteps", &Cartesian::getNDiscreteSteps, &Cartesian::setNDiscreteSteps,
		    "Number of discretization steps in the interval of definition\n"
		    "where configurations are computed.")
      .def("timeRange", &Cartesian::getTimeRange,
	   "Get interval of definition of right hand side of trajectory constraint.")
      .def("setRightHandSide", &Cartesian::setRightHandSide1,
	   "Set right hand side from a hpp::core::Path\n"
	   "\n:param rhs:function from an interval to SE(3).\n:param se3Output:set to True if the output of path must be understood as SE3.")
      .def("setRightHandSide", &Cartesian::setRightHandSide2,
	   "Set right hand side from a hpp:constraints::DifferentiableFunction\n"
	   "\n:param rhs function.\n:param timeRange interval of definition of the function.")
      .def("getRightHandSide", &Cartesian::getRightHandSide,
	   "Get right hand side function of trajectory constraint.")
      .def("planPath", &Cartesian::planPath, "Plan a path starting from an initial configuration.\n"
	   "\n"
	   ":param q_init:initial configuration\n"
	   ":return: (success, result) where\n"
	   "    result is the resulting path in case of success, a valid portion of path satisfying\n"
	   "    the trajectory constraint along a sub-interval starting at 0 otherwise.\n"
	   "\n"
	   "The successive steps of the computations are the following.\n"
	   "  - The interval of definition is discretized into a number of sub-intervals defined by\n"
	   "    method \link Cartesian::nDiscreteSteps\n"
	   "    nDiscreteSteps\endlink. For each discretized value, a configuration is computed by\n"
	   "    projecting the previous one onto the time-varying constraint or the initial\n"
	   "    configuration for the first discretized value.\n"
	   "  - the path interpolating these configurations and associated to the time-varying\n"
	   "    constraint is tested for collision. If no collision is detected, the function\n"
	   "    returns true.\n"
	   "\n"
	   "In case of failure in the first step, the interpolated path until the last successful\n"
	   "projection is returned without collision checking.\n"
	   "In case of failure in the second step, a collision-free path defined over a sub-interval\n"
	   "starting at 0 and satisfying the constraints is returned.");
    boost::python::def("makePiecewiseLinearTrajectory",
	hpp::manipulation::steeringMethod::Cartesian::makePiecewiseLinearTrajectory,
	"Build a piecewise linear path.\n"
        "See C++ documentation of class hpp::manipulation::steeringMethod::Cartesian.");
  }
}  // namespace manipulation
}  // namespace pyhpp
} // namespace steeringMethod
