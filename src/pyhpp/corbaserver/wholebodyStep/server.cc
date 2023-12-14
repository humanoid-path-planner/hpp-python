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
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/wholebody-step/server.hh>
#include <hpp/core/problem-solver.hh>
#include <pyhpp/corbaserver/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace corbaserver {
namespace wholebodyStep {
typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::wholebodyStep::Server WholebodyServer;

struct SWrapper {
  static WholebodyServer* init(CorbaServer* server, bool multithread = false) {
    WholebodyServer* s = new WholebodyServer(0, NULL, multithread);
    s->setProblemSolverMap(server->problemSolverMap());
    return s;
  }
  static void startCorbaServer(WholebodyServer* wbs,
                               const CorbaServer* server) {
    wbs->startCorbaServer(server->mainContextId(), "corbaserver",
                          "wholebodyStep", "problem");
  }
};

void exposeServer() {
  class_<WholebodyServer>("Server", no_init)
      .def("__init__", make_constructor(&SWrapper::init))
          .PYHPP_DEFINE_METHOD(SWrapper, startCorbaServer);
}
}  // namespace wholebodyStep
}  // namespace corbaserver
}  // namespace pyhpp
