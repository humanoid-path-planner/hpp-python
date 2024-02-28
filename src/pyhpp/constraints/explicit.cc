//
// Copyright (c) 2023 CNRS
// Authors: Florent Lamiraux
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
#include <hpp/constraints/explicit.hh>
#include <pinocchio/multibody/fwd.hpp>

using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

Eigen::BlockIndex::segments_t toSegments(const std::vector<size_type>& in) {
  segments_t out;
  out.reserve(in.size() / 2);
  for (std::size_t i = 0; i < in.size(); i += 2)
    out.push_back(Eigen::BlockIndex::segment_t(in[i], in[i + 1]));
  return out;
}
Eigen::BlockIndex::segments_t toSegments(const list& in) {
  segments_t out(len(in));
  for (int i = 0; i < len(in); ++i)
    out[i] = extract<Eigen::BlockIndex::segment_t>(in[i]);
  return out;
}

ExplicitPtr_t createExplicit(const LiegroupSpacePtr_t& configSpace,
                             const DifferentiableFunctionPtr_t& f,
                             const list& inArg, const list& outArg,
                             const list& inDer, const list& outDer,
                             const ComparisonTypes_t& comp) {
  return Explicit::create(configSpace, f, toSegments(inArg), toSegments(outArg),
                          toSegments(inDer), toSegments(outDer), comp);
}

void exposeExplicit() {
  class_<Explicit, ExplicitPtr_t, boost::noncopyable>("Explicit", no_init)
      .def("create", &createExplicit)
      .staticmethod("create");
}
}  // namespace constraints
}  // namespace pyhpp
