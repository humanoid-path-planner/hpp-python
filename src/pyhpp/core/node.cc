//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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
