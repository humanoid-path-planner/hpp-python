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

#ifndef PYHPP_REF_HH
#define PYHPP_REF_HH

#include <eigenpy/eigenpy.hpp>
#include <hpp/pinocchio/fwd.hh>

namespace pyhpp {
typedef eigenpy::Ref<hpp::pinocchio::vector_t> vectorRef_t;
typedef eigenpy::Ref<hpp::pinocchio::vector_t const> vectorConstRef_t;
typedef eigenpy::Ref<hpp::pinocchio::matrix_t> matrixRef_t;
typedef eigenpy::Ref<hpp::pinocchio::matrix_t const> matrixConstRef_t;
}  // namespace pyhpp

#endif  // PYHPP_REF_HH
