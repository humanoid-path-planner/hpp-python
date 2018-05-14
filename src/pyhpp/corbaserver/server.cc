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

#include <pyhpp/corbaserver/fwd.hh>

#include <boost/python.hpp>

#include <hpp/corbaserver/server.hh>

#include <hpp/core/problem-solver.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace corbaserver {
    using hpp::core::ProblemSolverPtr_t;
    using namespace hpp::corbaServer;

    struct SWrapper {
      static Server* init1 (ProblemSolverPtr_t ps, bool multithread = false)
      {
        return new Server (ps, 0, NULL, multithread);
      }
      static Server* init2 (ProblemSolverPtr_t ps, const char* name, bool multithread = false)
      {
        const char** argv = new const char*[2];
        const char* arg0 = "--name";
        argv[0] = arg0;
        argv[1] = name;
        Server* server = new Server (ps, 2, argv, multithread);
        delete[] argv;
        return server;
      }
    };

    void exposeServer ()
    {
      class_ <Server> ("Server", no_init)
        // .def (init<hpp::core::ProblemSolverPtr_t, int, const char* [], bool>())
        .def ("__init__", make_constructor(&SWrapper::init1))
        .def ("__init__", make_constructor(&SWrapper::init2))
        // .def (init<ProblemSolverMapPtr_t, int, const char* [], bool>)
        PYHPP_DEFINE_METHOD (Server, startCorbaServer)
        PYHPP_DEFINE_METHOD (Server, processRequest)
        PYHPP_DEFINE_METHOD_INTERNAL_REF (Server, mainContextId)
        // PYHPP_DEFINE_METHOD (Server, problemSolver)
        // PYHPP_DEFINE_METHOD (Server, problemSolverMap)
        ;
    }
  }
}
