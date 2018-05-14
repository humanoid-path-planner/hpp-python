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

#include <pyhpp/util.hh>
#include <pyhpp/core/fwd.hh>

BOOST_PYTHON_MODULE(bindings)
{
  boost::python::import ("pyhpp.constraints");

  pyhpp::core::exposeProblem();
  pyhpp::core::exposeProblemSolver();

  pyhpp::core::exposeConfigurationShooter();
  pyhpp::core::exposeConstraint();
  pyhpp::core::exposeEquation();
  pyhpp::core::exposeReports();
  pyhpp::core::exposeSteeringMethod();

  // Expose main abstract classes
  pyhpp::core::exposePath();
  pyhpp::core::exposePathOptimizer();
  pyhpp::core::exposePathValidation();

  boost::python::import ("pyhpp.core.path");
  boost::python::import ("pyhpp.core.pathoptimization");
}
