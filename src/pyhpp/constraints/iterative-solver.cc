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
#include <hpp/constraints/implicit-constraint-set.hh>
#include <hpp/constraints/solver/hierarchical-iterative.hh>
// cland-format on

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <pyhpp/constraints/fwd.hh>
#include <pyhpp/util.hh>

// DocNamespace(hpp::constraints)

namespace {
const char* DOC_HI_ERRORTHRESHOLD = "Get error threshold.";
const char* DOC_HI_MAXITERATIONS =
    "Get maximal number of iterations in config projector.";
const char* DOC_HI_RHSFC1 =
    "Compute right hand side of equality constraints from a configuration.\n\n"
    "For each constraint of type Equality, set right hand side as rhs = f(q).\n"
    "Only parameterizable constraints (type Equality) are set.";
const char* DOC_HI_RHSFC2 =
    "Compute right hand side of a constraint from a configuration.\n\n"
    "Set right hand side as rhs = f(q).\n"
    "Only parameterizable constraints (type Equality) are set.";
const char* DOC_HI_RHS_SET1 =
    "Set right hand side of a constraint.\n\n"
    "Size of rhs should be equal to the total dimension of parameterizable\n"
    "constraints (type Equality).";
const char* DOC_HI_RHS_SET2 =
    "Set the right hand side.\n\n"
    "Size of rhs should be equal to the total dimension of parameterizable\n"
    "constraints (type Equality).";
const char* DOC_HI_RHS_GET =
    "Get the right hand side.\n\n"
    "Size of result is equal to total dimension of parameterizable\n"
    "constraints (type Equality).";
}  // namespace

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;
using namespace hpp::constraints::solver;

static boost::python::list getConstraintsForPriority(HierarchicalIterative& hi,
                                                     std::size_t priority) {
  if (priority >= hi.numberStacks()) {
    PyErr_SetString(PyExc_IndexError, "priority is out of range");
    boost::python::throw_error_already_set();
  }
  boost::python::list result;
  for (const auto& c : hi.constraints(priority).constraints()) result.append(c);
  return result;
}

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
              &HierarchicalIterative::errorThreshold),
          DOC_HI_ERRORTHRESHOLD)
      .def("rightHandSideFromConfig",
           static_cast<vector_t (HierarchicalIterative::*)(ConfigurationIn_t)>(
               &HierarchicalIterative::rightHandSideFromConfig),
           DOC_HI_RHSFC1)
      .def("rightHandSideFromConfig",
           static_cast<bool (HierarchicalIterative::*)(const ImplicitPtr_t&,
                                                       ConfigurationIn_t)>(
               &HierarchicalIterative::rightHandSideFromConfig),
           DOC_HI_RHSFC2)
      .def("rightHandSide",
           static_cast<bool (HierarchicalIterative::*)(const ImplicitPtr_t&,
                                                       vectorIn_t)>(
               &HierarchicalIterative::rightHandSide),
           DOC_HI_RHS_SET1)
      .def("rightHandSide",
           static_cast<void (HierarchicalIterative::*)(vectorIn_t)>(
               &HierarchicalIterative::rightHandSide),
           DOC_HI_RHS_SET2)
      .def("rightHandSide",
           static_cast<vector_t (HierarchicalIterative::*)() const>(
               &HierarchicalIterative::rightHandSide),
           DOC_HI_RHS_GET)
      .add_property("maxIterations",
                    static_cast<size_type (HierarchicalIterative::*)() const>(
                        &HierarchicalIterative::maxIterations),
                    static_cast<void (HierarchicalIterative::*)(size_type)>(
                        &HierarchicalIterative::maxIterations),
                    DOC_HI_MAXITERATIONS)
      .add_property(
          "errorThreshold",
          static_cast<value_type (HierarchicalIterative::*)() const>(
              &HierarchicalIterative::errorThreshold),
          static_cast<void (HierarchicalIterative::*)(const value_type&)>(
              &HierarchicalIterative::errorThreshold))
      .add_property("lastIsOptional",
                    static_cast<bool (HierarchicalIterative::*)() const>(
                        &HierarchicalIterative::lastIsOptional),
                    static_cast<void (HierarchicalIterative::*)(bool)>(
                        &HierarchicalIterative::lastIsOptional))
      .add_property("solveLevelByLevel",
                    static_cast<bool (HierarchicalIterative::*)() const>(
                        &HierarchicalIterative::solveLevelByLevel),
                    static_cast<void (HierarchicalIterative::*)(bool)>(
                        &HierarchicalIterative::solveLevelByLevel))
      .def("numberStacks", &HierarchicalIterative::numberStacks)
      .def("constraintsForPriority", &getConstraintsForPriority)
      .def("dimension", &HierarchicalIterative::dimension,
           return_value_policy<copy_const_reference>());
}
}  // namespace constraints
}  // namespace pyhpp
