//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
#include <hpp/core/connected-component.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace core {

using namespace hpp::core;

void exposeNode() {
  class_<Node, boost::shared_ptr<Node>, boost::noncopyable>("Node", no_init)
      .def(init<ConfigurationIn_t>())
      .def(init<ConfigurationIn_t, ConnectedComponentPtr_t>())
      .PYHPP_DEFINE_METHOD(Node, addOutEdge)
      .PYHPP_DEFINE_METHOD(Node, addInEdge)
      .def("connectedComponent",
           static_cast<ConnectedComponentPtr_t (Node::*)() const>(
               &Node::connectedComponent))
      .def("connectedComponent",
           static_cast<void (Node::*)(const ConnectedComponentPtr_t&)>(
               &Node::connectedComponent))
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Node, outEdges)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Node, inEdges)
      .PYHPP_DEFINE_METHOD(Node, isOutNeighbor)
      .PYHPP_DEFINE_METHOD(Node, isInNeighbor)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Node, configuration)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Node, print);
}
}  // namespace core
}  // namespace pyhpp
