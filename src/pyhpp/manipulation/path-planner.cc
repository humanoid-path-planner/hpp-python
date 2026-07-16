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

namespace {

const char* DOC_INNERPLANNER_GET = "Get the inner planner.";
const char* DOC_INNERPLANNER_SET = "Set the inner planner.";

const char* DOC_NRANDOMCONFIG =
    "Get the number of random configurations used to generate the initial "
    "config of the final path.";

const char* DOC_NDISCRETESTEPS =
    "Number of steps to generate goal config (successive projections).";

const char* DOC_CHECKFEASIBILITYONLY =
    "If enabled, only add one solution to the roadmap. "
    "Otherwise add all solutions.";

template <typename Planner>
void plannerStartSolve(Planner& planner) {
  planner.obj->startSolve();
}

template <typename Planner>
void plannerTryConnectInitAndGoals(Planner& planner) {
  planner.obj->tryConnectInitAndGoals();
}

template <typename Planner>
void plannerOneStep(Planner& planner) {
  planner.obj->oneStep();
}

}  // namespace

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
  PyErr_WarnEx(PyExc_DeprecationWarning,
               "planPath is deprecated, use computePath()", 1);
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
  // Workaround for eigenpy bug: (1,N) numpy arrays have both C- and
  // F-contiguous flags set. eigenpy's is_arr_layout_compatible_with_mat_type
  // sees F-contiguous and creates Ref<MatrixXd> with Stride<0,0>, causing
  // the actual numpy strides to be ignored. Only element (0,0) maps correctly;
  // all other columns receive garbage. Re-map via the raw data pointer with an
  // explicit RowMajor layout to recover the correct values.
  // Multi-row matrices (rows > 1) are not affected: their C- and F-contiguous
  // flags differ, so eigenpy correctly allocates a copy.
  if (qGoals.rows() == 1) {
    typedef Eigen::Map<
        const Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>
        RowMap;
    const hpp::constraints::matrix_t goals =
        RowMap(qGoals.data(), 1, qGoals.cols());
    return this->computePath(qInit, goals.transpose().eval(), resetRoadmap);
  }
  return this->computePath(qInit, qGoals.transpose().eval(), resetRoadmap);
}

PathVectorPtr_t TransitionPlanner::computePath(ConfigurationIn_t qInit,
                                               matrixIn_t qGoals,
                                               bool resetRoadmap) {
  if (qInit.rows() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "qInit = " << hpp::pinocchio::displayConfig(qInit)
       << "should be of size " << obj->problem()->robot()->configSize() << ".";
    throw std::logic_error(os.str().c_str());
  }
  if (qGoals.rows() != obj->problem()->robot()->configSize()) {
    std::ostringstream os;
    os << "qGoals = " << qGoals << "should have "
       << obj->problem()->robot()->configSize() << " rows.";
    throw std::logic_error(os.str().c_str());
  }
  if (qGoals.cols() < 1) {
    std::ostringstream os;
    os << "qGoals = " << qGoals << "should have at least one line.";
    throw std::logic_error(os.str().c_str());
  }
  // Workaround for eigenpy bug: (N,1) numpy arrays have both C- and
  // F-contiguous flags set. eigenpy's is_arr_layout_compatible_with_mat_type
  // sees C-contiguous and creates Ref<MatrixXd> with Stride<0,0>, causing
  // the actual numpy strides to be ignored. Only element (0,0) maps correctly;
  // all other rows receive garbage. Re-map via the raw data pointer with an
  // explicit ColMajor layout to recover the correct values.
  // Multi-column matrices (cols > 1) are not affected: their C- and
  // F-contiguous flags differ, so eigenpy correctly allocates a copy.
  if (qGoals.cols() == 1) {
    typedef Eigen::Map<
        const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>
        ColMap;
    const hpp::constraints::matrix_t goals =
        ColMap(qGoals.data(), qGoals.rows(), 1);
    return trObj()->computePath(qInit, goals, resetRoadmap);
  }
  return trObj()->computePath(qInit, qGoals, resetRoadmap);
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
  warnings.attr("warn")(
      "pyhpp.manipulation.TransitionPlanner.setEdge is deprecated. "
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
  boost::python::object warnings = boost::python::import("warnings");
  warnings.attr("warn")(
      "pyhpp.manipulation.EndEffectorTrajectory is deprecated. "
      "Use pyhpp.manipulation.steering_method.Cartesian instead.");
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
      "TransitionPlanner", DocClassDoc(),
      boost::python::init<const pyhpp::core::Problem&>())
      .def("startSolve", &plannerStartSolve<TransitionPlanner>,
           DocClassMethod(startSolve))
      .def("oneStep", &plannerOneStep<TransitionPlanner>,
           DocClassMethod(oneStep))
      .def("innerPlanner",
           static_cast<pyhpp::core::PathPlanner (TransitionPlanner::*)() const>(
               &TransitionPlanner::innerPlanner),
           DOC_INNERPLANNER_GET)
      .def("innerPlanner",
           static_cast<void (TransitionPlanner::*)(
               const pyhpp::core::PathPlanner&)>(
               &TransitionPlanner::innerPlanner),
           DOC_INNERPLANNER_SET)
      .def("innerProblem", &TransitionPlanner::innerProblem,
           DocClassMethod(innerProblem))
      .def("computePath", &TransitionPlanner::computePath,
           DocClassMethod(computePath))
      .def("planPath", &TransitionPlanner::planPath, DocClassMethod(planPath))
      .def("directPath", &TransitionPlanner::directPath,
           "Compute a direct path on a transition. Returns (success, path, "
           "status).")
      .def("validateConfiguration", &TransitionPlanner::validateConfiguration,
           "Validate configuration against the graph state identified by id. "
           "Returns (valid, report).")
      .def("optimizePath", &TransitionPlanner::optimizePath,
           DocClassMethod(optimizePath))
      .def("timeParameterization", &TransitionPlanner::timeParameterization,
           DocClassMethod(timeParameterization))
      .def("setEdge", &TransitionPlanner::setEdge,
           DocClassMethod(setEdge))  // deprecated
      .def("setTransition", &TransitionPlanner::setTransition,
           DocClassMethod(setEdge))
      .def("setReedsAndSheppSteeringMethod",
           &TransitionPlanner::setReedsAndSheppSteeringMethod,
           DocClassMethod(setReedsAndSheppSteeringMethod))
      .def("pathProjector", &TransitionPlanner::pathProjector,
           DocClassMethod(pathProjector))
      .def("clearPathOptimizers", &TransitionPlanner::clearPathOptimizers,
           DocClassMethod(clearPathOptimizers))
      .def("addPathOptimizer", &TransitionPlanner::addPathOptimizer,
           DocClassMethod(addPathOptimizer));

  // DocNamespace(hpp::manipulation)
  // DocClass(ManipulationPlanner)
  boost::python::class_<ManipulationPlanner,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "ManipulationPlanner", DocClassDoc(),
      boost::python::init<const pyhpp::core::Problem&>())
      .def("oneStep", &plannerOneStep<ManipulationPlanner>,
           DocClassMethod(oneStep));

  // DocNamespace(hpp::manipulation::pathPlanner)
  // DocClass(StatesPathFinder)
  boost::python::class_<StatesPathFinder,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "StatesPathFinder", DocClassDoc(),
      boost::python::init<const pyhpp::core::Problem&>())
      .def("startSolve", &plannerStartSolve<StatesPathFinder>,
           DocClassMethod(startSolve))
      .def("tryConnectInitAndGoals",
           &plannerTryConnectInitAndGoals<StatesPathFinder>,
           DocClassMethod(tryConnectInitAndGoals))
      .def("oneStep", &plannerOneStep<StatesPathFinder>,
           DocClassMethod(oneStep));

  // DocClass(EndEffectorTrajectory)
  boost::python::class_<EndEffectorTrajectory,
                        boost::python::bases<pyhpp::core::PathPlanner>>(
      "EndEffectorTrajectory", DocClassDoc(),
      boost::python::init<const pyhpp::core::Problem&>())
      .def(boost::python::init<const pyhpp::core::Problem&,
                               const RoadmapPtr_t&>())
      .def("startSolve", &plannerStartSolve<EndEffectorTrajectory>,
           DocClassMethod(startSolve))
      .def("tryConnectInitAndGoals",
           &plannerTryConnectInitAndGoals<EndEffectorTrajectory>,
           DocClassMethod(tryConnectInitAndGoals))
      .def("oneStep", &plannerOneStep<EndEffectorTrajectory>,
           DocClassMethod(oneStep))
      .def("nRandomConfig",
           static_cast<int (EndEffectorTrajectory::*)() const>(
               &EndEffectorTrajectory::nRandomConfig),
           DOC_NRANDOMCONFIG)
      .def("nRandomConfig", static_cast<void (EndEffectorTrajectory::*)(int)>(
                                &EndEffectorTrajectory::nRandomConfig))
      .def("nDiscreteSteps",
           static_cast<int (EndEffectorTrajectory::*)() const>(
               &EndEffectorTrajectory::nDiscreteSteps),
           DOC_NDISCRETESTEPS)
      .def("nDiscreteSteps", static_cast<void (EndEffectorTrajectory::*)(int)>(
                                 &EndEffectorTrajectory::nDiscreteSteps))
      .def("checkFeasibilityOnly",
           static_cast<bool (EndEffectorTrajectory::*)() const>(
               &EndEffectorTrajectory::checkFeasibilityOnly),
           DOC_CHECKFEASIBILITYONLY)
      .def("checkFeasibilityOnly",
           static_cast<void (EndEffectorTrajectory::*)(bool)>(
               &EndEffectorTrajectory::checkFeasibilityOnly))
      // .def("ikSolverInitialization",
      // &EndEffectorTrajectory::ikSolverInitialization)
      ;
}

}  // namespace manipulation
}  // namespace pyhpp
