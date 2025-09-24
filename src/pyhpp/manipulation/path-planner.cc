//
// Copyright (c) 2025, CNRS
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

#include <../src/pyhpp/manipulation/graph.hh>
#include <../src/pyhpp/manipulation/path-planner.hh>
#include <hpp/manipulation/path-planner/transition-planner.hh>
#include <pyhpp/core/path-planner.hh>
#include <pyhpp/core/problem.hh>

namespace pyhpp {
namespace manipulation {

void exposePathPlanners() {
  boost::python::class_<TransitionPlanner, boost::python::bases<pyhpp::core::PathPlanner> > (
      "TransitionPlanner", boost::python::init<const pyhpp::core::Problem&>())
    .def("innerPlanner", static_cast<pyhpp::core::PathPlanner (TransitionPlanner::*) () const> (
        &TransitionPlanner::innerPlanner))
    .def("innerPlanner", static_cast<void (TransitionPlanner::*)(const pyhpp::core::PathPlanner&)> (
	&TransitionPlanner::innerPlanner))
    .def("innerProblem", &TransitionPlanner::innerProblem)
    .def("planPath", &TransitionPlanner::planPath)
    .def("directPath", &TransitionPlanner::directPath)
    .def("validateConfiguration", &TransitionPlanner::validateConfiguration)
    .def("optimizePath", &TransitionPlanner::optimizePath)
    .def("timeParameterization", &TransitionPlanner::timeParameterization)
    .def("setEdge", &TransitionPlanner::setEdge)
    .def("setReedsAndSheppSteeringMethod", &TransitionPlanner::setReedsAndSheppSteeringMethod)
    .def("pathProjector", &TransitionPlanner::pathProjector)
    .def("clearPathOptimizers", &TransitionPlanner::clearPathOptimizers)
    .def("addPathOptimizer", &TransitionPlanner::addPathOptimizer);
	
}

TransitionPlanner::TransitionPlanner(const pyhpp::core::Problem& problem)
{
  obj = hpp::manipulation::pathPlanner::TransitionPlanner::createWithRoadmap(
      problem.obj, hpp::core::Roadmap::create(problem.obj->distance(), problem.obj->robot())
  );
}

pyhpp::core::PathPlanner TransitionPlanner::innerPlanner() const {
  pyhpp::core::PathPlanner pathPlanner;
  pathPlanner.obj = obj->innerPlanner();
  return pathPlanner;
}

void TransitionPlanner::innerPlanner(const pyhpp::core::PathPlanner& planner) {
  obj->innerPlanner(planner.obj);
}
pyhpp::core::Problem TransitionPlanner::innerProblem() const {
  return pyhpp::core::Problem(obj->innerProblem());
}

PathVectorPtr_t TransitionPlanner::planPath(ConfigurationIn_t qInit, matrixIn_t qGoals,
					    bool resetRoadmap) {
  return obj->planPath(qInit, qGoals, resetRoadmap);
}

tuple TransitionPlanner::directPath(ConfigurationIn_t q1, ConfigurationIn_t q2,
		 bool validate) {
  bool success;
  std::string status;
  PathPtr_t path = obj->directPath(q1, q2, validate, success, status);
  return boost::python::make_tuple(success, path, status);
}

tuple TransitionPlanner::validateConfiguration(ConfigurationIn_t q, std::size_t id) const {
  hpp::core::ValidationReportPtr_t report;
  bool res = obj->validateConfiguration(q, id, report);
  return boost::python::make_tuple(res, report);
}

PathVectorPtr_t TransitionPlanner::optimizePath(const PathPtr_t& path) {
  return obj->optimizePath(path);
}

PathVectorPtr_t TransitionPlanner::timeParameterization(const PathVectorPtr_t& path) {
  return obj->timeParameterization(path);
}

void TransitionPlanner::setEdge(const PyWEdge& transition) {
  obj->setEdge(transition.obj);
}

void TransitionPlanner::setReedsAndSheppSteeringMethod(double turningRadius) {
  obj->setReedsAndSheppSteeringMethod(turningRadius);
}

void TransitionPlanner::pathProjector(const PathProjectorPtr_t pathProjector) {
  obj->pathProjector(pathProjector);
}

void TransitionPlanner::clearPathOptimizers() {
  obj->clearPathOptimizers();
}

void TransitionPlanner::addPathOptimizer(const PathOptimizerPtr_t& pathOptimizer) {
  obj->addPathOptimizer(pathOptimizer);
}

}  // namespace manipulation
}  // namespace pyhpp

