//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
//
// This file is part of hpp-python
// hpp-python is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python  If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/diffusing-planner.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-planner/bi-rrt-star.hh>
#include <hpp/core/path-planner/k-prm-star.hh>
#include <hpp/core/path-planner/search-in-roadmap.hh>
#include <hpp/core/plan-and-optimize.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/visibility-prm-planner.hh>
#include <pyhpp/core/path-planner.hh>

namespace pyhpp {
namespace core {

using namespace boost::python;

namespace pathPlanner {

#define DEFINE_PLANNER_WRAPPER(WrapperName, PlannerType, PlannerPtr) \
  struct WrapperName : public pyhpp::core::PathPlanner {             \
    WrapperName(const pyhpp::core::Problem& problem) {               \
      obj = PlannerType::create(problem.obj);                        \
    }                                                                \
  };                                                                 \
  void expose##WrapperName() {                                       \
    class_<WrapperName, bases<pyhpp::core::PathPlanner>>(            \
        #WrapperName, init<const pyhpp::core::Problem&>());          \
  }

DEFINE_PLANNER_WRAPPER(DiffusingPlanner, hpp::core::DiffusingPlanner,
                       hpp::core::DiffusingPlannerPtr_t)
DEFINE_PLANNER_WRAPPER(BiRRTPlanner, hpp::core::BiRRTPlanner,
                       hpp::core::BiRRTPlannerPtr_t)
DEFINE_PLANNER_WRAPPER(VisibilityPrmPlanner, hpp::core::VisibilityPrmPlanner,
                       hpp::core::VisibilityPrmPlannerPtr_t)
DEFINE_PLANNER_WRAPPER(BiRrtStar, hpp::core::pathPlanner::BiRrtStar,
                       hpp::core::pathPlanner::BiRrtStarPtr_t)
DEFINE_PLANNER_WRAPPER(kPrmStar, hpp::core::pathPlanner::kPrmStar,
                       hpp::core::pathPlanner::kPrmStarPtr_t)

struct SearchInRoadmap : public pyhpp::core::PathPlanner {
  SearchInRoadmap(const pyhpp::core::Problem& problem,
                  const RoadmapPtr_t& roadmap) {
    obj = hpp::core::pathPlanner::SearchInRoadmap::createWithRoadmap(
        problem.obj, roadmap);
  }
};

void exposeSearchInRoadmap() {
  class_<SearchInRoadmap, bases<pyhpp::core::PathPlanner>>(
      "SearchInRoadmap",
      init<const pyhpp::core::Problem&, const hpp::core::RoadmapPtr_t&>());
}

struct PlanAndOptimize : public pyhpp::core::PathPlanner {
  PlanAndOptimize(const pyhpp::core::PathPlanner& pathPlanner) {
    obj = hpp::core::PlanAndOptimize::create(pathPlanner.obj);
  }
};

void exposePlanAndOptimize() {
  class_<PlanAndOptimize, bases<pyhpp::core::PathPlanner>>(
      "PlanAndOptimize", init<const pyhpp::core::PathPlanner&>());
}

void exposePathPlanners() {
  pyhpp::core::pathPlanner::exposeDiffusingPlanner();
  pyhpp::core::pathPlanner::exposeBiRRTPlanner();
  pyhpp::core::pathPlanner::exposeVisibilityPrmPlanner();
  pyhpp::core::pathPlanner::exposeBiRrtStar();
  pyhpp::core::pathPlanner::exposeSearchInRoadmap();
  pyhpp::core::pathPlanner::exposekPrmStar();
  pyhpp::core::pathPlanner::exposePlanAndOptimize();
}
}  // namespace pathPlanner

// Wrapper methods
const RoadmapPtr_t& PathPlanner::roadmap() const { return obj->roadmap(); }

ProblemConstPtr_t PathPlanner::problem() const { return obj->problem(); }

void PathPlanner::startSolve() { obj->startSolve(); }

PathVectorPtr_t PathPlanner::solve() { return obj->solve(); }

void PathPlanner::tryConnectInitAndGoals() { obj->tryConnectInitAndGoals(); }

void PathPlanner::oneStep() { obj->oneStep(); }

PathVectorPtr_t PathPlanner::finishSolve(const PathVectorPtr_t& path) {
  return obj->finishSolve(path);
}

void PathPlanner::interrupt() { obj->interrupt(); }

void PathPlanner::maxIterations(const unsigned long int& n) {
  obj->maxIterations(n);
}

unsigned long int PathPlanner::maxIterations() const {
  return obj->maxIterations();
}

void PathPlanner::timeOut(const double& timeOut) { obj->timeOut(timeOut); }

double PathPlanner::timeOut() const { return obj->timeOut(); }

void PathPlanner::stopWhenProblemIsSolved(bool enable) {
  obj->stopWhenProblemIsSolved(enable);
}

PathVectorPtr_t PathPlanner::computePath() const { return obj->computePath(); }

void exposePathPlanner() {
  class_<PathPlanner>("PathPlanner", no_init)
      .PYHPP_DEFINE_METHOD_CONST_REF(PathPlanner, roadmap)
      .PYHPP_DEFINE_METHOD(PathPlanner, problem)
      .PYHPP_DEFINE_METHOD(PathPlanner, startSolve)
      .PYHPP_DEFINE_METHOD(PathPlanner, solve)
      .PYHPP_DEFINE_METHOD(PathPlanner, tryConnectInitAndGoals)
      .PYHPP_DEFINE_METHOD(PathPlanner, oneStep)
      .PYHPP_DEFINE_METHOD(PathPlanner, finishSolve)
      .PYHPP_DEFINE_METHOD(PathPlanner, interrupt)
      .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(PathPlanner, maxIterations,
                                            unsigned long int)
      .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(PathPlanner, timeOut, double)
      .PYHPP_DEFINE_METHOD(PathPlanner, stopWhenProblemIsSolved)
      .PYHPP_DEFINE_METHOD(PathPlanner, computePath);

  pyhpp::core::pathPlanner::exposePathPlanners();
}

}  // namespace core
}  // namespace pyhpp
