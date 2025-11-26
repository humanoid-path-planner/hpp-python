//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
//
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
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <hpp/core/connected-component.hh>
#include <hpp/core/problem.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

struct CCWrapper {
  static boost::python::list nodes(ConnectedComponent& cc) {
    Configurations_t res;
    for (const auto& n : cc.nodes()) {
      res.push_back(n->configuration());
    }
    return to_python_list(res);
  }
  static boost::python::list reachableFrom(ConnectedComponent& cc) {
    std::vector<ConnectedComponentPtr_t> res;
    for (ConnectedComponent::RawPtrs_t::const_iterator itcc =
             cc.reachableFrom().begin();
         itcc != cc.reachableFrom().end(); ++itcc) {
      res.push_back((*itcc)->self());
    }
    return to_python_list(res);
  }
  static boost::python::list reachableTo(ConnectedComponent& cc) {
    std::vector<ConnectedComponentPtr_t> res;
    for (ConnectedComponent::RawPtrs_t::const_iterator itcc =
             cc.reachableTo().begin();
         itcc != cc.reachableTo().end(); ++itcc) {
      res.push_back((*itcc)->self());
    }
    return to_python_list(res);
  }
  // Test that raw pointers are the same.
  static bool equality(ConnectedComponent& cc1, ConnectedComponent& cc2) {
    return &cc1 == &cc2;
  }
};
void exposeConnectedComponent() {
  class_<ConnectedComponent, ConnectedComponentPtr_t, boost::noncopyable>(
      "ConnectedComponent", no_init)
      .def("nodes", &CCWrapper::nodes)
      .def("reachableFrom", &CCWrapper::reachableFrom)
      .def("reachableTo", &CCWrapper::reachableTo)
      .def("__eq__", &CCWrapper::equality);
}
}  // namespace core
}  // namespace pyhpp
