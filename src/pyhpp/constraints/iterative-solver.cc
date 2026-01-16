//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
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

// cland-format off
#include <hpp/constraints/solver/hierarchical-iterative.hh>
// cland-format on

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <pyhpp/constraints/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::constraints)

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;
using namespace hpp::constraints::solver;

void exposeHierarchicalIterativeSolver() {
  class_<ComparisonTypes_t>("ComparisonTypes")
      .def(vector_indexing_suite<ComparisonTypes_t>());

  // DocClass(solver::HierarchicalIterative)
  class_<HierarchicalIterative>("HierarchicalIterative",
                                init<LiegroupSpacePtr_t>())
      .def("__str__", &to_str<HierarchicalIterative>)
      .def("add", &HierarchicalIterative::add, DocClassMethod(add))

      .add_property(
          "errorThreshold",
          static_cast<value_type (HierarchicalIterative::*)() const>(
              &HierarchicalIterative::errorThreshold),
          static_cast<void (HierarchicalIterative::*)(const value_type&)>(
              &HierarchicalIterative::errorThreshold))
      .add_property("maxIterations",
                    static_cast<size_type (HierarchicalIterative::*)() const>(
                        &HierarchicalIterative::maxIterations),
                    static_cast<void (HierarchicalIterative::*)(size_type)>(
                        &HierarchicalIterative::maxIterations));
}
}  // namespace constraints
}  // namespace pyhpp
