//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
//
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

#include <pyhpp/constraints/fwd.hh>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <hpp/constraints/iterative-solver.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace constraints {
    using namespace hpp::constraints;

    void exposeHierarchicalIterativeSolver ()
    {
      enum_ <ComparisonType> ("ComparisonType")
        .value ("Equality"    , Equality   )
        .value ("EqualToZero" , EqualToZero)
        .value ("Superior"    , Superior   )
        .value ("Inferior"    , Inferior   )
        ;
      class_< ComparisonTypes_t >("ComparisonTypes")
        .def (vector_indexing_suite <ComparisonTypes_t> ())
        ;

      class_<HierarchicalIterativeSolver> ("HierarchicalIterativeSolver", init<std::size_t, std::size_t>())
        .def ("__str__", &to_str<HierarchicalIterativeSolver>)
        .def ("add",
            static_cast <void (HierarchicalIterativeSolver::*) (const DifferentiableFunctionPtr_t&, const std::size_t&)>
            (&HierarchicalIterativeSolver::add))
        .def ("add",
            static_cast <void (HierarchicalIterativeSolver::*) (const DifferentiableFunctionPtr_t&, const std::size_t&, const ComparisonTypes_t&)>
            (&HierarchicalIterativeSolver::add))

        .add_property ("errorThreshold",
            static_cast <value_type (HierarchicalIterativeSolver::*) () const     > (&HierarchicalIterativeSolver::errorThreshold),
            static_cast <void (HierarchicalIterativeSolver::*) (const value_type&)> (&HierarchicalIterativeSolver::errorThreshold))
        .add_property ("maxIterations",
            static_cast <size_type (HierarchicalIterativeSolver::*) () const> (&HierarchicalIterativeSolver::maxIterations),
            static_cast <void (HierarchicalIterativeSolver::*) (size_type)  > (&HierarchicalIterativeSolver::maxIterations))
        ;
    }
  }
}
