// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <boost/python.hpp>
#include <hpp/core/path-projector.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct PPWrapper {
  static bool apply(PathProjector* pp, const PathPtr_t path,
                    PathPtr_t& projPath) {
    return pp->apply(path, projPath);
  }

  static tuple py_apply(PathProjector* pp, const PathPtr_t path) {
    PathPtr_t projPath;
    bool res = pp->apply(path, projPath);
    return boost::python::make_tuple(res, projPath);
  }
};

void exposePathProjector() {
  class_<PathProjector, PathProjectorPtr_t, boost::noncopyable>("PathProjector",
                                                                no_init)
      .def("apply", &PPWrapper::apply)

      .def("apply", &PPWrapper::py_apply);
}
}  // namespace core
}  // namespace pyhpp
