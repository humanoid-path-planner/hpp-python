//
// Copyright (c) 2023 CNRS
// Authors: Florent Lamiraux
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

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
