//
// Copyright (c) 2018 - 2023 CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#ifndef PYHPP_REF_HH
#define PYHPP_REF_HH

#include <eigenpy/eigenpy.hpp>
#include <hpp/constraints/fwd.hh>

namespace pyhpp {
typedef hpp::constraints::vectorOut_t vectorRef_t;
typedef hpp::constraints::vectorIn_t vectorConstRef_t;
typedef hpp::constraints::matrixOut_t matrixRef_t;
typedef hpp::constraints::matrixIn_t matrixConstRef_t;
}  // namespace pyhpp

#endif  // PYHPP_REF_HH
