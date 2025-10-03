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
#include <hpp/core/problem-solver.hh>
#include <pyhpp/corbaserver/fwd.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace corbaserver {
using hpp::core::ProblemSolverPtr_t;
using namespace hpp::corbaServer;

struct SWrapper {
  static Server* init1(ProblemSolverPtr_t ps, bool multithread = false) {
    return new Server(ps, 0, NULL, multithread);
  }
  static Server* init2(ProblemSolverPtr_t ps, const char* name,
                       bool multithread = false) {
    const char** argv = new const char*[2];
    const char* arg0 = "--name";
    argv[0] = arg0;
    argv[1] = name;
    Server* server = new Server(ps, 2, argv, multithread);
    delete[] argv;
    return server;
  }
};

void exposeServer() {
  class_<Server, boost::noncopyable>("Server", init<ProblemSolverPtr_t, bool>())
      .def("__init__", make_constructor(&SWrapper::init1))
      .def("__init__", make_constructor(&SWrapper::init2))
      .def("initialize", &Server::initialize)
      .PYHPP_DEFINE_METHOD(Server, startCorbaServer)
      .PYHPP_DEFINE_METHOD(Server, processRequest)
      .PYHPP_DEFINE_METHOD_INTERNAL_REF(Server, mainContextId);
}
}  // namespace corbaserver
}  // namespace pyhpp
