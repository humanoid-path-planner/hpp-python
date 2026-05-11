//
// Copyright (c) 2025, CNRS
// Authors: Paul Sardin
//
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
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/problem.hh>
#include <hpp/manipulation/graph-optimizer.hh>
#include <hpp/manipulation/path-optimization/enforce-transition-semantic.hh>
#include <hpp/manipulation/path-optimization/random-shortcut.hh>

// DocNamespace(hpp::manipulation)

using namespace boost::python;

namespace {
const char* DOC_ENFORCE_TRANSITION_SEMANTIC =
  "Recompute the transition relative to each element of the path vector\n"
  "\n"
  "When executing a sequence of direct paths on a real robot, it is useful to know which\n"
  "transition of the graph each direct path corresponds to. For example, in a manipulation\n"
  "motion, before grasping an object, the robot needs to open the gripper. This information\n"
  "is contained in the transition that leads to a pregrasp waypoint state. The direct path\n"
  "should therefore have access to the transition.\n"
  "\n"
  "The information is stored in the hpp::manipulation::ConstraintSet.\n"
  "of the path and is accessible via method hpp::manipulation::ConstraintSet::edge\n"
  "\n"
  "If the path vector is produced by a manipulation planner, each direct path has been created\n"
  "by a transition. However, the path may later be cut by random shortcut or due to collision and\n"
  "the associated transition become irrelevant. For example if a path is created by a transition\n"
  "that leads to a pre-grasp, and cut due to a collision, the path does not reach the target\n"
  "state and the relevant transition is not the one that built the path.\n"
  "\n"
  "This class takes a hpp::core::PathVector as input an relabel\n"
  "each direct path with the correct transition.\n"
  "\n"
  "Precondition: The path should have been created by a manipulation planning algorithm: in other\n"
  "words, the constraint set of each direct path should be of type\n"
  "hpp::manipulation::ConstraintSet.";
}
namespace pyhpp {
namespace manipulation {
using namespace hpp::manipulation;

template <typename InnerOpt>
hpp::core::PathOptimizerPtr_t createGraphOptimizer(
    const hpp::core::ProblemConstPtr_t& problem) {
  return GraphOptimizer::create<InnerOpt>(problem);
}

void exposePathOptimizers() {
  class_<pathOptimization::RandomShortcut,
         std::shared_ptr<pathOptimization::RandomShortcut>,
         bases<hpp::core::PathOptimizer>, boost::noncopyable>("RandomShortcut",
                                                              no_init)
      .def("__init__",
           make_constructor(&pathOptimization::RandomShortcut::create));

  class_<pathOptimization::EnforceTransitionSemantic,
         std::shared_ptr<pathOptimization::EnforceTransitionSemantic>,
         bases<hpp::core::PathOptimizer>, boost::noncopyable>(
         "EnforceTransitionSemantic", DOC_ENFORCE_TRANSITION_SEMANTIC, no_init)
      .def("__init__",
           make_constructor(
               &pathOptimization::EnforceTransitionSemantic::create));

  def("GraphRandomShortcut",
      &createGraphOptimizer<hpp::core::pathOptimization::RandomShortcut>);

  def("GraphPartialShortcut",
      &createGraphOptimizer<hpp::core::pathOptimization::PartialShortcut>);
}

}  // namespace manipulation
}  // namespace pyhpp
