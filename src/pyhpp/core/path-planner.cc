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

// DocNamespace(hpp::core)

namespace {
const char* DOC_PP_MAXITER_GET = "Get maximal number of iterations.";
const char* DOC_PP_MAXITER_SET = "Set maximal number of iterations.";
const char* DOC_PP_TIMEOUT_GET = "Get time out.";
const char* DOC_PP_TIMEOUT_SET = "Set time out (in seconds).";
}  // namespace

namespace pyhpp {
namespace core {

using namespace boost::python;

namespace pathPlanner {

#define DEFINE_PLANNER_WRAPPER(WrapperName, PlannerType) \
  struct WrapperName : public pyhpp::core::PathPlanner { \
    WrapperName(const pyhpp::core::Problem& problem) {   \
      obj = PlannerType::create(problem.obj);            \
    }                                                    \
  }

DEFINE_PLANNER_WRAPPER(DiffusingPlanner, hpp::core::DiffusingPlanner);
DEFINE_PLANNER_WRAPPER(BiRRTPlanner, hpp::core::BiRRTPlanner);
DEFINE_PLANNER_WRAPPER(VisibilityPrmPlanner, hpp::core::VisibilityPrmPlanner);
DEFINE_PLANNER_WRAPPER(BiRrtStar, hpp::core::pathPlanner::BiRrtStar);
DEFINE_PLANNER_WRAPPER(kPrmStar, hpp::core::pathPlanner::kPrmStar);

// DocClass(DiffusingPlanner)
void exposeDiffusingPlanner() {
  class_<DiffusingPlanner, bases<pyhpp::core::PathPlanner>>(
      "DiffusingPlanner", DocClassDoc(), init<const pyhpp::core::Problem&>())
      .def("startSolve", &pyhpp::core::PathPlanner::startSolve,
           DocClassMethod(startSolve))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

// DocClass(BiRRTPlanner)
void exposeBiRRTPlanner() {
  class_<BiRRTPlanner, bases<pyhpp::core::PathPlanner>>(
      "BiRRTPlanner", DocClassDoc(), init<const pyhpp::core::Problem&>())
      .def("startSolve", &pyhpp::core::PathPlanner::startSolve,
           DocClassMethod(startSolve))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

// DocClass(VisibilityPrmPlanner)
void exposeVisibilityPrmPlanner() {
  class_<VisibilityPrmPlanner, bases<pyhpp::core::PathPlanner>>(
      "VisibilityPrmPlanner", DocClassDoc(),
      init<const pyhpp::core::Problem&>())
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

// DocClass(pathPlanner::BiRrtStar)
void exposeBiRrtStar() {
  class_<BiRrtStar, bases<pyhpp::core::PathPlanner>>(
      "BiRrtStar", DocClassDoc(), init<const pyhpp::core::Problem&>())
      .def("startSolve", &pyhpp::core::PathPlanner::startSolve,
           DocClassMethod(startSolve))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

// DocClass(pathPlanner::kPrmStar)
void exposekPrmStar() {
  class_<kPrmStar, bases<pyhpp::core::PathPlanner>>(
      "kPrmStar", DocClassDoc(), init<const pyhpp::core::Problem&>())
      .def("startSolve", &pyhpp::core::PathPlanner::startSolve,
           DocClassMethod(startSolve))
      .def("tryConnectInitAndGoals",
           &pyhpp::core::PathPlanner::tryConnectInitAndGoals,
           DocClassMethod(tryConnectInitAndGoals))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

struct SearchInRoadmap : public pyhpp::core::PathPlanner {
  SearchInRoadmap(const pyhpp::core::Problem& problem,
                  const RoadmapPtr_t& roadmap) {
    obj = hpp::core::pathPlanner::SearchInRoadmap::createWithRoadmap(
        problem.obj, roadmap);
  }
};

// DocClass(pathPlanner::SearchInRoadmap)
void exposeSearchInRoadmap() {
  class_<SearchInRoadmap, bases<pyhpp::core::PathPlanner>>(
      "SearchInRoadmap", DocClassDoc(),
      init<const pyhpp::core::Problem&, const hpp::core::RoadmapPtr_t&>())
      .def("tryConnectInitAndGoals",
           &pyhpp::core::PathPlanner::tryConnectInitAndGoals,
           DocClassMethod(tryConnectInitAndGoals))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep));
}

struct PlanAndOptimize : public pyhpp::core::PathPlanner {
  PlanAndOptimize(const pyhpp::core::PathPlanner& pathPlanner) {
    obj = hpp::core::PlanAndOptimize::create(pathPlanner.obj);
  }
};

// DocClass(PlanAndOptimize)
void exposePlanAndOptimize() {
  class_<PlanAndOptimize, bases<pyhpp::core::PathPlanner>>(
      "PlanAndOptimize", DocClassDoc(), init<const pyhpp::core::PathPlanner&>())
      .def("startSolve", &pyhpp::core::PathPlanner::startSolve,
           DocClassMethod(startSolve))
      .def("oneStep", &pyhpp::core::PathPlanner::oneStep,
           DocClassMethod(oneStep))
      .def("finishSolve", &pyhpp::core::PathPlanner::finishSolve,
           DocClassMethod(finishSolve));
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
  // DocClass(PathPlanner)
  class_<PathPlanner>("PathPlanner", DocClassDoc(), no_init)
      .def("roadmap", &PathPlanner::roadmap,
           return_value_policy<copy_const_reference>(), DocClassMethod(roadmap))
      .def("problem", &PathPlanner::problem, DocClassMethod(problem))
      .def("startSolve", &PathPlanner::startSolve, DocClassMethod(startSolve))
      .def("solve", &PathPlanner::solve, DocClassMethod(solve))
      .def("tryConnectInitAndGoals", &PathPlanner::tryConnectInitAndGoals,
           DocClassMethod(tryConnectInitAndGoals))
      .def("oneStep", &PathPlanner::oneStep, DocClassMethod(oneStep))
      .def("finishSolve", &PathPlanner::finishSolve,
           DocClassMethod(finishSolve))
      .def("interrupt", &PathPlanner::interrupt, DocClassMethod(interrupt))
      .def("maxIterations",
           static_cast<unsigned long int (PathPlanner::*)() const>(
               &PathPlanner::maxIterations),
           DOC_PP_MAXITER_GET)
      .def("maxIterations",
           static_cast<void (PathPlanner::*)(const unsigned long int&)>(
               &PathPlanner::maxIterations),
           DOC_PP_MAXITER_SET)
      .def("timeOut",
           static_cast<double (PathPlanner::*)() const>(&PathPlanner::timeOut),
           DOC_PP_TIMEOUT_GET)
      .def("timeOut",
           static_cast<void (PathPlanner::*)(const double&)>(
               &PathPlanner::timeOut),
           DOC_PP_TIMEOUT_SET)
      .def("stopWhenProblemIsSolved", &PathPlanner::stopWhenProblemIsSolved,
           DocClassMethod(stopWhenProblemIsSolved))
      .def("computePath", &PathPlanner::computePath,
           DocClassMethod(computePath));

  pyhpp::core::pathPlanner::exposePathPlanners();
}

}  // namespace core
}  // namespace pyhpp
