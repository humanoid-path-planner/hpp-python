//
// Copyright (c) 2025 CNRS
// Authors: Paul Sardin
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

#include <hpp/constraints/relative-com.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
using namespace boost::python;

namespace pyhpp {
namespace constraints {
using namespace hpp::constraints;

static RelativeComPtr_t create1(  std::string& name,
                                     const DevicePtr_t& robot,
                                     const JointPtr_t& joint,
                                     const vector3_t reference,
                                     std::vector<bool> mask) {
  CenterOfMassComputationPtr_t comc = CenterOfMassComputation::create(robot);
  comc->add(robot->rootJoint());
  return RelativeCom::create(name, robot, comc, joint, reference, mask);
}
static RelativeComPtr_t create2(const DevicePtr_t& robot,
                                     const CenterOfMassComputationPtr_t& comc,
                                     const JointPtr_t& joint,
                                     const vector3_t reference,
                                     std::vector<bool> mask) {
  return RelativeCom::create("RelativeCom", robot, comc, joint, reference, mask);
}
static RelativeComPtr_t create3(const std::string& name,
                                     const DevicePtr_t& robot,
                                     const CenterOfMassComputationPtr_t& comc,
                                     const JointPtr_t& joint,
                                     const vector3_t reference,
                                     std::vector<bool> mask) {
  return RelativeCom::create(name, robot, comc, joint, reference, mask);
}

void exposeRelativeCom() {
class_<RelativeCom, RelativeComPtr_t, bases<DifferentiableFunction>, boost::noncopyable>("RelativeCom", no_init)      
.def("create", &create1)
      .def("create", &create2)
      .def("create", &create3).staticmethod("create");
}
}  // namespace constraints
}  // namespace pyhpp
