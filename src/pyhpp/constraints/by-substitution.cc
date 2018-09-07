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

#include <hpp/constraints/solver/by-substitution.hh>

using namespace boost::python;

namespace pyhpp {
  namespace constraints {
    using namespace hpp::constraints;
    using namespace hpp::constraints::solver;

    tuple BySubstitution_solve (const BySubstitution& hs, const vector_t& q)
    {
      vector_t qout (q);
      HierarchicalIterative::Status s = hs.solve(qout);
      return make_tuple (qout, s);
    }

    void exposeBySubstitution ()
    {
      class_<BySubstitution, bases<HierarchicalIterative> > ("BySubstitution", init<LiegroupSpacePtr_t>())
        .def ("explicitConstraintSet", static_cast <ExplicitConstraintSet& (BySubstitution::*) ()> (&BySubstitution::explicitConstraintSet),
            return_internal_reference<>())
        .def ("explicitConstraintSetHasChanged", &BySubstitution::explicitConstraintSetHasChanged)
        .def ("solve", &BySubstitution_solve)

        .add_property ("errorThreshold",
            static_cast <value_type (BySubstitution::*) () const     > (&BySubstitution::errorThreshold),
            static_cast <void (BySubstitution::*) (const value_type&)> (&BySubstitution::errorThreshold))
        ;
    }
  }
}
