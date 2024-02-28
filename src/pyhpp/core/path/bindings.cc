//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <pyhpp/core/path/fwd.hh>
#include <pyhpp/util.hh>

BOOST_PYTHON_MODULE(bindings) {
  INIT_PYHPP_MODULE;

  pyhpp::core::path::exposeSplines();
  pyhpp::core::path::exposeVector();
}
