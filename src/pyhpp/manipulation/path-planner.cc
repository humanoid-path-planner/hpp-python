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
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/path-planner/end-effector-trajectory.hh>
#include <hpp/manipulation/path-planner/states-path-finder.hh>
#include <hpp/manipulation/path-planner/transition-planner.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/pinocchio/configuration.hh>
#include <pyhpp/core/path-planner.hh>
#include <pyhpp/core/problem.hh>

// DocNamespace(hpp::manipulation::pathPlanner)

namespace pyhpp {
namespace manipulation {

struct ManipulationPlanner : public pyhpp::core::PathPlanner {
  ManipulationPlanner(const pyhpp::core::Problem& problem) {
    hpp::manipulation::RoadmapPtr_t roadmap =
        hpp::manipulation::Roadmap::create(problem.obj->distance(),
                                           problem.obj->robot());
    obj = hpp::manipulation::ManipulationPlanner::create(problem.obj, roadmap);
    roadmap->constraintGraph(
        problem.asManipulationProblem()->constraintGraph());
  }
};

struct StatesPathFinder : public pyhpp::core::PathPlanner {
  StatesPathFinder(const pyhpp::core::Problem& problem) {
    hpp::manipulation::RoadmapPtr_t roadmap =
        hpp::manipulation::Roadmap::create(problem.obj->distance(),
                                           problem.obj->robot());
    obj = hpp::manipulation::pathPlanner::StatesPathFinder::createWithRoadmap(
        problem.obj, roadmap);
    roadmap->constraintGraph(
        problem.asManipulationProblem()->constraintGraph());
  }
};

// TransitionPlanner implementation
TransitionPlanner::TransitionPlanner(const pyhpp::core::Problem& problem) {
  obj = hpp::manipulation::pathPlanner::TransitionPlanner::createWithRoadmap(
      problem.obj, hpp::core::Roadmap::create(problem.obj->distance(),
                                              problem.obj->robot()));
}

hpp::manipulation::pathPlanner::TransitionPlannerPtr_t
TransitionPlanner::trObj() const {
  assert(HPP_DYNAMIC_PTR_CAST(hpp::manipulation::pathPlanner::TransitionPlanner,
                              obj));
  return HPP_STATIC_PTR_CAST(hpp::manipulation::pathPlanner::TransitionPlanner,
                             obj);
}

pyhpp::core::PathPlanner TransitionPlanner::innerPlanner() const {
  pyhpp::core::PathPlanner pathPlanner;
  pathPlanner.obj = trObj()->innerPlanner();
  return pathPlanner;
}

void TransitionPlanner::innerPlanner(const pyhpp::core::PathPlanner& planner) {
  trObj()->innerPlanner(planner.obj);
}

pyhpp::core::Problem TransitionPlanner::innerProblem() const {
  return pyhpp::core::Problem(trObj()->innerProblem());
}

PathVectorPtr_t TransitionPlanner::planPath(ConfigurationIn_t qInit,
                                            matrixIn_t qGoals,
                                            bool resetRoadmap) {
  if (qInit.rows() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "qInit = " << hpp::pinocchio::displayConfig(qInit)
       << "should be of size " << obj->problem()->robot()->configSize() << ".";
    throw std::logic_error(os.str().c_str());
  }
  if (qGoals.cols() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "qGoals = " << qGoals << "should have "
       << obj->problem()->robot()->configSize() << " columns.";
    throw std::logic_error(os.str().c_str());
  }
  if (qGoals.rows() < 1) {
    std::ostringstream os;
    os << "qGoals = " << qGoals << "should have at least one line.";
    throw std::logic_error(os.str().c_str());
  }
  return trObj()->planPath(qInit, qGoals, resetRoadmap);
}

tuple TransitionPlanner::directPath(ConfigurationIn_t q1, ConfigurationIn_t q2,
                                    bool validate) {
  if (q1.rows() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "q1 = " << hpp::pinocchio::displayConfig(q1) << "should be of size "
       << obj->problem()->robot()->configSize() << ".";
    throw std::logic_error(os.str().c_str());
  }
  if (q2.rows() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "q2 = " << hpp::pinocchio::displayConfig(q2) << "should be of size "
       << obj->problem()->robot()->configSize() << ".";
    throw std::logic_error(os.str().c_str());
  }
  bool success;
  std::string status;
  PathPtr_t path = trObj()->directPath(q1, q2, validate, success, status);
  return boost::python::make_tuple(success, path, status);
}

tuple TransitionPlanner::validateConfiguration(ConfigurationIn_t q,
                                               std::size_t id) const {
  hpp::core::ValidationReportPtr_t report;
  bool res = trObj()->validateConfiguration(q, id, report);
  return boost::python::make_tuple(res, report);
}

PathVectorPtr_t TransitionPlanner::optimizePath(const PathPtr_t& path) {
  return trObj()->optimizePath(path);
}

PathVectorPtr_t TransitionPlanner::timeParameterization(
    const PathVectorPtr_t& path) {
  return trObj()->timeParameterization(path);
}

// deprecated
void TransitionPlanner::setEdge(const PyWEdge& transition) {
  trObj()->setEdge(transition.obj);
  boost::python::object warnings = boost::python::import("warnings");
  warnings.attr("warn")("pyhpp.manipulation.TransitionPlanner.setEdge is deprecated. "
                        "Use setTransition instead.");
}

void TransitionPlanner::setTransition(const PyWEdge& transition) {
  trObj()->setEdge(transition.obj);
}

void TransitionPlanner::setReedsAndSheppSteeringMethod(double turningRadius) {
  trObj()->setReedsAndSheppSteeringMethod(turningRadius);
}

void TransitionPlanner::pathProjector(const PathProjectorPtr_t pathProjector) {
  trObj()->pathProjector(pathProjector);
}

void TransitionPlanner::clearPathOptimizers() {
  trObj()->clearPathOptimizers();
}

void TransitionPlanner::addPathOptimizer(
    const PathOptimizerPtr_t& pathOptimizer) {
  trObj()->addPathOptimizer(pathOptimizer);
}

// EndEffectorTrajectory implementation
EndEffectorTrajectory::EndEffectorTrajectory(
    const pyhpp::core::Problem& problem) {
  obj =
      hpp::manipulation::pathPlanner::EndEffectorTrajectory::createWithRoadmap(
          problem.obj, hpp::core::Roadmap::create(problem.obj->distance(),
                                                  problem.obj->robot()));
}

EndEffectorTrajectory::EndEffectorTrajectory(
    const pyhpp::core::Problem& problem,
    const hpp::core::RoadmapPtr_t& roadmap) {
  obj =
      hpp::manipulation::pathPlanner::EndEffectorTrajectory::createWithRoadmap(
          problem.obj, roadmap);
}

hpp::manipulation::pathPlanner::EndEffectorTrajectoryPtr_t
EndEffectorTrajectory::eetObj() const {
  assert(HPP_DYNAMIC_PTR_CAST(
      hpp::manipulation::pathPlanner::EndEffectorTrajectory, obj));
  return HPP_STATIC_PTR_CAST(
      hpp::manipulation::pathPlanner::EndEffectorTrajectory, obj);
}

int EndEffectorTrajectory::nRandomConfig() const {
  return eetObj()->nRandomConfig();
}

void EndEffectorTrajectory::nRandomConfig(int n) { eetObj()->nRandomConfig(n); }

int EndEffectorTrajectory::nDiscreteSteps() const {
  return eetObj()->nDiscreteSteps();
}

void EndEffectorTrajectory::nDiscreteSteps(int n) {
  eetObj()->nDiscreteSteps(n);
}

void EndEffectorTrajectory::checkFeasibilityOnly(bool enable) {
  eetObj()->checkFeasibilityOnly(enable);
}

bool EndEffectorTrajectory::checkFeasibilityOnly() const {
  return eetObj()->checkFeasibilityOnly();
}

// void
// EndEffectorTrajectory::ikSolverInitialization(IkSolverInitializationPtr_t
// solver) {
//   eetObj()->ikSolverInitialization(solver);
// }

void exposePathPlanners() {
  // DocClass(TransitionPlanner)
  boost::python::class_<TransitionPlanner,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "TransitionPlanner", boost::python::init<const pyhpp::core::Problem&>())
      .def("innerPlanner",
           static_cast<pyhpp::core::PathPlanner (TransitionPlanner::*)() const>(
               &TransitionPlanner::innerPlanner))
      .def("innerPlanner", static_cast<void (TransitionPlanner::*)(
                               const pyhpp::core::PathPlanner&)>(
                               &TransitionPlanner::innerPlanner))
      .def("innerProblem", &TransitionPlanner::innerProblem,
           DocClassMethod(innerProblem))
      .def("planPath", &TransitionPlanner::planPath, DocClassMethod(planPath))
      .def("directPath", &TransitionPlanner::directPath)
      .def("validateConfiguration", &TransitionPlanner::validateConfiguration)
      .def("optimizePath", &TransitionPlanner::optimizePath,
           DocClassMethod(optimizePath))
      .def("timeParameterization", &TransitionPlanner::timeParameterization,
           DocClassMethod(timeParameterization))
      .def("setEdge", &TransitionPlanner::setEdge, DocClassMethod(setEdge)) // deprecated
      .def("setTransition", &TransitionPlanner::setTransition, DocClassMethod(setEdge))
      .def("setReedsAndSheppSteeringMethod",
           &TransitionPlanner::setReedsAndSheppSteeringMethod,
           DocClassMethod(setReedsAndSheppSteeringMethod))
      .def("pathProjector", &TransitionPlanner::pathProjector,
           DocClassMethod(pathProjector))
      .def("clearPathOptimizers", &TransitionPlanner::clearPathOptimizers,
           DocClassMethod(clearPathOptimizers))
      .def("addPathOptimizer", &TransitionPlanner::addPathOptimizer,
           DocClassMethod(addPathOptimizer));

  boost::python::class_<ManipulationPlanner,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "ManipulationPlanner",
      boost::python::init<const pyhpp::core::Problem&>());

  boost::python::class_<StatesPathFinder,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "StatesPathFinder", boost::python::init<const pyhpp::core::Problem&>());

  // DocClass(EndEffectorTrajectory)
  boost::python::class_<EndEffectorTrajectory,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "EndEffectorTrajectory",
      boost::python::init<const pyhpp::core::Problem&>())
      .def(boost::python::init<const pyhpp::core::Problem&,
                               const RoadmapPtr_t&>())
      .def("nRandomConfig", static_cast<int (EndEffectorTrajectory::*)() const>(
                                &EndEffectorTrajectory::nRandomConfig))
      .def("nRandomConfig", static_cast<void (EndEffectorTrajectory::*)(int)>(
                                &EndEffectorTrajectory::nRandomConfig))
      .def("nDiscreteSteps",
           static_cast<int (EndEffectorTrajectory::*)() const>(
               &EndEffectorTrajectory::nDiscreteSteps))
      .def("nDiscreteSteps", static_cast<void (EndEffectorTrajectory::*)(int)>(
                                 &EndEffectorTrajectory::nDiscreteSteps))
      .def("checkFeasibilityOnly",
           static_cast<bool (EndEffectorTrajectory::*)() const>(
               &EndEffectorTrajectory::checkFeasibilityOnly))
      .def("checkFeasibilityOnly",
           static_cast<void (EndEffectorTrajectory::*)(bool)>(
               &EndEffectorTrajectory::checkFeasibilityOnly))
      // .def("ikSolverInitialization",
      // &EndEffectorTrajectory::ikSolverInitialization)
      ;
}

}  // namespace manipulation
}  // namespace pyhpp
