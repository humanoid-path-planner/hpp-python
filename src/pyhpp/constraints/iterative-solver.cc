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

#include <hpp/constraints/solver/hierarchical-iterative.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace constraints {
    using namespace hpp::constraints;
    using namespace hpp::constraints::solver;

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

      class_<HierarchicalIterative> ("HierarchicalIterative", init<LiegroupSpacePtr_t>())
        .def ("__str__", &to_str<HierarchicalIterative>)
        .def ("add",
            static_cast <void (HierarchicalIterative::*) (const DifferentiableFunctionPtr_t&, const std::size_t&)>
            (&HierarchicalIterative::add))
        .def ("add",
            static_cast <void (HierarchicalIterative::*) (const DifferentiableFunctionPtr_t&, const std::size_t&, const ComparisonTypes_t&)>
            (&HierarchicalIterative::add))

        .add_property ("errorThreshold",
            static_cast <value_type (HierarchicalIterative::*) () const     > (&HierarchicalIterative::errorThreshold),
            static_cast <void (HierarchicalIterative::*) (const value_type&)> (&HierarchicalIterative::errorThreshold))
        .add_property ("maxIterations",
            static_cast <size_type (HierarchicalIterative::*) () const> (&HierarchicalIterative::maxIterations),
            static_cast <void (HierarchicalIterative::*) (size_type)  > (&HierarchicalIterative::maxIterations))
        ;
    }
  }
}
