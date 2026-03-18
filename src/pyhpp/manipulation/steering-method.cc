//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pyhpp/core/steering-method.hh>
#include <hpp/manipulation/steering-method/cartesian.hh>

// DocNamespace(hpp::manipulation)

namespace pyhpp {
namespace manipulation {

GraphSteeringMethod::GraphSteeringMethod(
    const PyWSteeringMethodPtr_t& steeringMethodWrapper) {
  hpp::manipulation::steeringMethod::GraphPtr_t graph =
      hpp::manipulation::steeringMethod::Graph::create(
          steeringMethodWrapper->obj->problem());
  graph->innerSteeringMethod(steeringMethodWrapper->obj);
  obj = graph;
}

hpp::manipulation::steeringMethod::EndEffectorTrajectoryPtr_t
EndEffectorTrajectorySteeringMethod::eetObj() const {
  assert(HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::steeringMethod::EndEffectorTrajectory, obj));
  return HPP_STATIC_PTR_CAST(
      hpp::manipulation::steeringMethod::EndEffectorTrajectory, obj);
}

EndEffectorTrajectorySteeringMethod::EndEffectorTrajectorySteeringMethod(
    const hpp::core::ProblemConstPtr_t& problem)
    : SteeringMethod(
          hpp::manipulation::steeringMethod::EndEffectorTrajectory::create(
              problem)) {}

void EndEffectorTrajectorySteeringMethod::setTrajectoryConstraint(
    const hpp::constraints::ImplicitPtr_t& ic) {
  eetObj()->trajectoryConstraint(ic);
}

void EndEffectorTrajectorySteeringMethod::setTrajectory(
    const hpp::core::PathPtr_t& eeTraj, bool se3Output) {
  eetObj()->trajectory(eeTraj, se3Output);
}

void exposeManipSteeringMethod() {
  class_<GraphSteeringMethod>(
      "GraphSteeringMethod",
      boost::python::init<const PyWSteeringMethodPtr_t&>());

  class_<EndEffectorTrajectorySteeringMethod,
         bases<pyhpp::core::SteeringMethod>>(
      "EndEffectorTrajectorySteeringMethod",
      boost::python::init<const hpp::core::ProblemConstPtr_t&>())
      .def("setTrajectoryConstraint",
           &EndEffectorTrajectorySteeringMethod::setTrajectoryConstraint)
      .def("setTrajectory",
           &EndEffectorTrajectorySteeringMethod::setTrajectory);
}

}  // namespace manipulation
}  // namespace pyhpp
