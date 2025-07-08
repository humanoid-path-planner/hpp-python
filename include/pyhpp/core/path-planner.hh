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

#ifndef PYHPP_CORE_PATH_PLANNER_HH
#define PYHPP_CORE_PATH_PLANNER_HH

#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

#include <pyhpp/core/problem.hh>

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

};  // struct PathPlanner

}  // namespace core
}  // namespace pyhpp

#endif
