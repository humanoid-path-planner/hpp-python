// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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
