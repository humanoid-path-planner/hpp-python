//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <pyhpp/core/fwd.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-vector.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

void exposePathOptimizer() {
  class_<PathOptimizer, PathOptimizerPtr_t, boost::noncopyable>("PathOptimizer",
                                                                no_init)
      .def("problem", &PathOptimizer::problem)
      .def("optimize", &PathOptimizer::optimize)
      .def("interrupt", &PathOptimizer::interrupt)
      .def("maxIterations", &PathOptimizer::maxIterations)
      .def("timeOut", &PathOptimizer::timeOut);

  class_<pathOptimization::RandomShortcut, std::shared_ptr<pathOptimization::RandomShortcut>, bases<PathOptimizer>,
        boost::noncopyable>("RandomShortcut", no_init)
      .def("__init__", make_constructor(&pathOptimization::RandomShortcut::create));


}
}  // namespace core
}  // namespace pyhpp
