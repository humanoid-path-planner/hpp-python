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
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-planner/bi-rrt-star.hh>
#include <hpp/core/diffusing-planner.hh>

#include <pyhpp/core/path-planner.hh>

namespace pyhpp {
namespace core {

using namespace boost::python;

namespace pathPlanner {

struct DiffusingPlanner : public pyhpp::core::PathPlanner {
    DiffusingPlanner(const hpp::core::DiffusingPlannerPtr_t& object)
    {
      obj = object;
    }
    DiffusingPlanner(const pyhpp::core::Problem& problem)
    {
      obj = hpp::core::DiffusingPlanner::create(problem.obj);
    }
};

void exposeDiffusingPlanner() {
  class_<DiffusingPlanner, bases<pyhpp::core::PathPlanner>>("DiffusingPlanner",
                                                             init <const pyhpp::core::Problem&> ());
}

}  // namespace pathPlanner

// Wrapper methods
const RoadmapPtr_t& PathPlanner::roadmap() const {
  return obj->roadmap();
}

ProblemConstPtr_t PathPlanner::problem() const {
  return obj->problem();
}

void PathPlanner::startSolve() {
  obj->startSolve();
}

PathVectorPtr_t PathPlanner::solve() {
  return obj->solve();
}

void PathPlanner::tryConnectInitAndGoals() {
  obj->tryConnectInitAndGoals();
}

void PathPlanner::oneStep() {
  obj->oneStep();
}

PathVectorPtr_t PathPlanner::finishSolve(const PathVectorPtr_t& path) {
  return obj->finishSolve(path);
}

void PathPlanner::interrupt() {
  obj->interrupt();
}

void PathPlanner::maxIterations(const unsigned long int& n) {
  obj->maxIterations(n);
}

unsigned long int PathPlanner::maxIterations() const {
  return obj->maxIterations();
}

void PathPlanner::timeOut(const double& timeOut) {
  obj->timeOut(timeOut);
}

double PathPlanner::timeOut() const {
  return obj->timeOut();
}

void PathPlanner::stopWhenProblemIsSolved(bool enable) {
  obj->stopWhenProblemIsSolved(enable);
}

PathVectorPtr_t PathPlanner::computePath() const {
  return obj->computePath();
}

void exposePathPlanner() {
  class_<PathPlanner>("PathPlanner", no_init)
      .def("roadmap", &PathPlanner::roadmap,
           return_value_policy<copy_const_reference>(), "Get roadmap")
      .def("problem", &PathPlanner::problem, "Get problem")
      .def("startSolve", &PathPlanner::startSolve)
      .def("solve", &PathPlanner::solve, "Solve the path planning problem")
      .def("tryConnectInitAndGoals", &PathPlanner::tryConnectInitAndGoals)
      .def("oneStep", &PathPlanner::oneStep)
      .def("finishSolve", &PathPlanner::finishSolve,
           "Post processing of the resulting path")
      .def("interrupt", &PathPlanner::interrupt, "Interrupt path planning")
      .def("maxIterations",
           static_cast<void (PathPlanner::*)(const unsigned long int&)>(
               &PathPlanner::maxIterations),
           "Set maximal number of iterations")
      .def("maxIterations",
           static_cast<unsigned long int (PathPlanner::*)() const>(
               &PathPlanner::maxIterations),
           "Get maximal number of iterations")
      .def("timeOut",
           static_cast<void (PathPlanner::*)(const double&)>(
               &PathPlanner::timeOut),
           "Set time out (in seconds)")
      .def("timeOut",
           static_cast<double (PathPlanner::*)() const>(&PathPlanner::timeOut),
           "Get time out")
      .def("stopWhenProblemIsSolved", &PathPlanner::stopWhenProblemIsSolved,
           "Enable/disable stopping when problem is solved")
      .def("computePath", &PathPlanner::computePath,
           "Find a path in the roadmap and transform it in trajectory");

  pyhpp::core::pathPlanner::exposeDiffusingPlanner();
}

}  // namespace core
}  // namespace pyhpp