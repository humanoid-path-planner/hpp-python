//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux, Paul Sardin
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

#ifndef PYHPP_MANIPULATION_PATH_PLANNER_HH
#define PYHPP_MANIPULATION_PATH_PLANNER_HH

#include <hpp/manipulation/fwd.hh>
#include <pyhpp/core/path-planner.hh>

namespace pyhpp {
namespace manipulation {

typedef boost::python::tuple tuple;
typedef hpp::core::ConfigurationIn_t ConfigurationIn_t;
typedef hpp::core::PathOptimizerPtr_t PathOptimizerPtr_t;
typedef hpp::core::PathProjectorPtr_t PathProjectorPtr_t;
typedef hpp::core::PathPtr_t PathPtr_t;
typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;
typedef hpp::core::RoadmapPtr_t RoadmapPtr_t;
typedef hpp::manipulation::matrixIn_t matrixIn_t;
// typedef hpp::manipulation::pathPlanner::IkSolverInitializationPtr_t
// IkSolverInitializationPtr_t;

struct TransitionPlanner : public pyhpp::core::PathPlanner {
  // Dynamic cast pointer into TransitionPlanner
  hpp::manipulation::pathPlanner::TransitionPlannerPtr_t trObj() const;
  TransitionPlanner(const pyhpp::core::Problem& problem);
  pyhpp::core::PathPlanner innerPlanner() const;
  void innerPlanner(const pyhpp::core::PathPlanner& planner);
  pyhpp::core::Problem innerProblem() const;
  PathVectorPtr_t planPath(ConfigurationIn_t qInit, matrixIn_t qGoals,
                           bool resetRoadmap);
  tuple directPath(ConfigurationIn_t q1, ConfigurationIn_t q2, bool validate);
  tuple validateConfiguration(ConfigurationIn_t q, std::size_t id) const;
  PathVectorPtr_t optimizePath(const PathPtr_t& path);
  PathVectorPtr_t timeParameterization(const PathVectorPtr_t& path);
  void setEdge(const PyWEdge& transition);
  void setTransition(const PyWEdge& transition);
  void setReedsAndSheppSteeringMethod(double turningRadius);
  void pathProjector(const PathProjectorPtr_t pathProjector);
  void clearPathOptimizers();
  void addPathOptimizer(const PathOptimizerPtr_t& pathOptimizer);
};

struct EndEffectorTrajectory : public pyhpp::core::PathPlanner {
  EndEffectorTrajectory(const pyhpp::core::Problem& problem);
  EndEffectorTrajectory(const pyhpp::core::Problem& problem,
                        const RoadmapPtr_t& roadmap);
  hpp::manipulation::pathPlanner::EndEffectorTrajectoryPtr_t eetObj() const;
  int nRandomConfig() const;
  void nRandomConfig(int n);
  int nDiscreteSteps() const;
  void nDiscreteSteps(int n);
  void checkFeasibilityOnly(bool enable);
  bool checkFeasibilityOnly() const;
  // void ikSolverInitialization(IkSolverInitializationPtr_t solver);
};

void exposePathPlanners();

}  // namespace manipulation
}  // namespace pyhpp
#endif  // PYHPP_MANIPULATION_PATH_PLANNER_HH
