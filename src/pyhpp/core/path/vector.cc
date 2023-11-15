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

#include <hpp/core/path-vector.hh>
#include <hpp/python/config.hh>
#include <pyhpp/core/path/fwd.hh>
#include <pyhpp/util.hh>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

using namespace boost::python;

namespace pyhpp {
namespace core {
namespace path {
using namespace hpp::core;

void exposeVector() {
  class_<PathVector, PathVectorPtr_t, bases<Path>, boost::noncopyable>("Vector",
                                                                       no_init)
    .def("create", static_cast<PathVectorPtr_t (*)(size_type, size_type)>
         (&PathVector::create))
      .staticmethod("create")
    PYHPP_DEFINE_METHOD(PathVector, numberPaths)
          PYHPP_DEFINE_METHOD(PathVector, pathAtRank)
              PYHPP_DEFINE_METHOD(PathVector, rankAtParam)
                  PYHPP_DEFINE_METHOD(PathVector, appendPath)
      // PYHPP_DEFINE_METHOD (PathVector, concatenate)
      PYHPP_DEFINE_METHOD(PathVector, flatten);

  class_<PathVectors_t>("Vectors").def(
      vector_indexing_suite<PathVectors_t, true>());
}
}  // namespace path
}  // namespace core
}  // namespace pyhpp
