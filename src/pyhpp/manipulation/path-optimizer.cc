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
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/manipulation/path-optimization/enforce-transition-semantic.hh>
#include <hpp/manipulation/path-optimization/random-shortcut.hh>
#include <hpp/manipulation/graph-optimizer.hh>

using namespace boost::python;


namespace pyhpp {
namespace manipulation {
using namespace hpp::manipulation;

template<typename InnerOpt>
hpp::core::PathOptimizerPtr_t createGraphOptimizer(const hpp::core::ProblemConstPtr_t& problem) {
  return GraphOptimizer::create<InnerOpt>(problem);
}

void exposePathOptimizers() {

  class_<pathOptimization::RandomShortcut, 
         std::shared_ptr<pathOptimization::RandomShortcut>, 
         bases<hpp::core::PathOptimizer>,
         boost::noncopyable>("RandomShortcut", no_init)
      .def("__init__", make_constructor(&pathOptimization::RandomShortcut::create));

  class_<pathOptimization::EnforceTransitionSemantic,
         std::shared_ptr<pathOptimization::EnforceTransitionSemantic>,
         bases<hpp::core::PathOptimizer>,
         boost::noncopyable>("EnforceTransitionSemantic", no_init)
      .def("__init__", make_constructor(&pathOptimization::EnforceTransitionSemantic::create));

  def("GraphRandomShortcut", 
      &createGraphOptimizer<hpp::core::pathOptimization::RandomShortcut>);
  
  def("GraphPartialShortcut",
      &createGraphOptimizer<hpp::core::pathOptimization::PartialShortcut>);
}

}  // namespace manipulation
}  // namespace pyhpp
