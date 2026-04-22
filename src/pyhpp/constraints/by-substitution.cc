//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <eigenpy/eigenpy.hpp>
#include <hpp/constraints/implicit-constraint-set.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <pyhpp/constraints/fwd.hh>

#include <set>

// DocNamespace(hpp::constraints::solver)

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;
using namespace hpp::constraints::solver;

tuple BySubstitution_solve(const BySubstitution& hs, const vector_t& q) {
  vector_t qout(q);
  HierarchicalIterative::Status s = hs.solve(qout);
  return make_tuple(qout, s);
}

boost::python::list BySubstitution_describeError(BySubstitution& solver,
                                                  vectorIn_t arg) {
  size_type implicitDim = solver.dimension();
  size_type explicitDim = solver.explicitConstraintSet().errorSize();
  vector_t error(implicitDim + explicitDim);
  bool satisfied = solver.isSatisfied(arg, error);
  (void)satisfied;

  boost::python::list result;
  size_type offset = 0;

  // Implicit constraints by priority level
  std::set<ImplicitPtr_t> implicitSet;
  for (std::size_t p = 0; p < solver.numberStacks(); ++p) {
    const auto& stack = solver.constraints(p);
    for (const auto& c : stack.constraints()) {
      implicitSet.insert(c);
      const DifferentiableFunction& f = c->function();
      size_type nv = f.outputDerivativeSize();
      vector_t errSlice = error.segment(offset, nv);
      result.append(boost::python::make_tuple(
          f.name(), errSlice, std::string("implicit"), static_cast<int>(p)));
      offset += nv;
    }
  }

  // Explicit constraints: those in numericalConstraints() not in any stack
  for (const auto& c : solver.numericalConstraints()) {
    if (implicitSet.count(c) == 0) {
      const DifferentiableFunction& f = c->function();
      size_type nv = f.outputDerivativeSize();
      vector_t errSlice = error.segment(offset, nv);
      result.append(boost::python::make_tuple(
          f.name(), errSlice, std::string("explicit"), -1));
      offset += nv;
    }
  }

  return result;
}

void exposeBySubstitution() {
  enum_<HierarchicalIterative::Status>("SolverStatus")
      .value("ERROR_INCREASED", HierarchicalIterative::ERROR_INCREASED)
      .value("MAX_ITERATION_REACHED",
             HierarchicalIterative::MAX_ITERATION_REACHED)
      .value("INFEASIBLE", HierarchicalIterative::INFEASIBLE)
      .value("SUCCESS", HierarchicalIterative::SUCCESS);

  // DocClass(BySubstitution)
  class_<BySubstitution, bases<HierarchicalIterative> >(
      "BySubstitution", init<LiegroupSpacePtr_t>())
      .def("explicitConstraintSetHasChanged",
           &BySubstitution::explicitConstraintSetHasChanged,
           DocClassMethod(explicitConstraintSetHasChanged))
      .def("solve", &BySubstitution_solve)
      .def("explicitConstraintSet",
           static_cast<ExplicitConstraintSet& (BySubstitution::*)()>(
               &BySubstitution::explicitConstraintSet),
           return_internal_reference<>(), DocClassMethod(explicitConstraintSet))
      .def("rightHandSideFromConfig",
           static_cast<vector_t (BySubstitution::*)(ConfigurationIn_t)>(
               &BySubstitution::rightHandSideFromConfig))
      .def("rightHandSideFromConfig",
           static_cast<bool (BySubstitution::*)(const ImplicitPtr_t&,
                                                ConfigurationIn_t)>(
               &BySubstitution::rightHandSideFromConfig))
      .def("rightHandSide", static_cast<bool (HierarchicalIterative::*)(
                                const ImplicitPtr_t&, vectorIn_t)>(
                                &HierarchicalIterative::rightHandSide))
      .def("rightHandSide",
           static_cast<void (HierarchicalIterative::*)(vectorIn_t)>(
               &HierarchicalIterative::rightHandSide))
      .def("rightHandSide",
           static_cast<vector_t (HierarchicalIterative::*)() const>(
               &HierarchicalIterative::rightHandSide))
      .add_property("errorThreshold",
                    static_cast<value_type (BySubstitution::*)() const>(
                        &BySubstitution::errorThreshold),
                    static_cast<void (BySubstitution::*)(const value_type&)>(
                        &BySubstitution::errorThreshold))
      .def("describeError", &BySubstitution_describeError);
}
}  // namespace constraints
}  // namespace pyhpp
