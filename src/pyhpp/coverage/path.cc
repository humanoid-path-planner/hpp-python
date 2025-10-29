//
// Copyright (c) 2025 CNRS
// Author: Paul Sardin
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/coverage/path.hh>
#include <hpp/coverage/cost.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/pinocchio/device.hh>
#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
namespace coverage {
using hpp::coverage::Path;
using hpp::core::PathPtr_t;
using hpp::core::PathVector;
using hpp::core::PathVectorPtr_t;

static PathPtr_t multiply(const PathPtr_t& p1, const PathPtr_t& p2) {
  Path ops;
  return ops.multiply(p1, p2);
}

static PathPtr_t createSpline(hpp::core::ConfigurationIn_t pose0,
                               hpp::core::ConfigurationIn_t pose1,
                               hpp::core::value_type length,
                               hpp::core::size_type order) {
  Path ops;
  return ops.createSpline(pose0, pose1, length, order);
}

static void setCost(hpp::core::Roadmap& roadmap,
                    const hpp::pinocchio::DevicePtr_t& robot,
                    const std::string& gripperName) {
  roadmap.cost(hpp::coverage::ToolRotation::create(robot, gripperName));
}

BOOST_PYTHON_MODULE(bindings) {
  boost::python::import("pyhpp.core");

  class_<Path, ::hpp::shared_ptr<Path>, boost::noncopyable>("CoveragePath", no_init)
      .def("create", +[]() { return ::hpp::shared_ptr<Path>(new Path()); })
      .staticmethod("create")
      .def("multiply", &Path::multiply)
      .def("createSpline", &Path::createSpline)
      .def("setCost", &setCost)
      .staticmethod("setCost");
}

}  // namespace coverage
}  // namespace pyhpp