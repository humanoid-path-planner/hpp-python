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

#include <boost/python.hpp>

#include <pyhpp/constraints/fwd.hh>

BOOST_PYTHON_MODULE(pyhppconstraints)
{
  pyhpp::constraints::exposeDifferentiableFunction();
  pyhpp::constraints::exposeGenericTransformations();

  pyhpp::constraints::exposeExplicitSolver ();
  pyhpp::constraints::exposeHierarchicalIterativeSolver();
  pyhpp::constraints::exposeHybridSolver();
}
