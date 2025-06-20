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

#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include "pyhpp/core/problem.hh"

namespace pyhpp {
namespace core {
  typedef hpp::core::RoadmapPtr_t RoadmapPtr_t;
  typedef hpp::core::ProblemConstPtr_t ProblemConstPtr_t;
  typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;

struct PathPlanner {
    hpp::core::PathPlannerPtr_t obj;
    
    // Methods from hpp::core::PathPlanner
    const RoadmapPtr_t& roadmap() const;
    ProblemConstPtr_t problem() const;
    void startSolve();
    PathVectorPtr_t solve();
    void tryConnectInitAndGoals();
    void oneStep();
    PathVectorPtr_t finishSolve(const PathVectorPtr_t& path);
    void interrupt();
    void maxIterations(const unsigned long int& n);
    unsigned long int maxIterations() const;
    void timeOut(const double& timeOut);
    double timeOut() const;
    void stopWhenProblemIsSolved(bool enable);
    PathVectorPtr_t computePath() const;
    
}; // struct PathPlanner

} // namespace core
} // namespace pyhpp
