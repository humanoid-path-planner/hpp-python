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

#ifndef PYHPP_CORE_FWD_HH
#define PYHPP_CORE_FWD_HH

namespace pyhpp {
namespace core {
void exposeConfigValidation();
void exposeConfigurationShooter();
void exposeConstraint();

void exposeEquation();

void exposeReports();

void exposeSteeringMethod();

void exposePath();
void exposePathOptimizer();
void exposePathProjector();
void exposePathValidation();
void exposeProblem();
void exposeProblemSolver();
}  // namespace core
}  // namespace pyhpp

#endif  // PYHPP_CORE_FWD_HH
