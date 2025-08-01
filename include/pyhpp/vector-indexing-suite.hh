//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
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

#ifndef PYHPP_VECTOR_INDEXING_SUITE_HH
#define PYHPP_VECTOR_INDEXING_SUITE_HH

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace pyhpp {
template <class Container, bool NoProxy = false>
class cpp_like_vector_indexing_suite
    : public boost::python::vector_indexing_suite<
          Container, NoProxy,
          cpp_like_vector_indexing_suite<Container, NoProxy> > {
 public:
  typedef boost::python::vector_indexing_suite<Container, NoProxy,
                                               cpp_like_vector_indexing_suite>
      base_type;
  template <class Class>
  static void extension_def(Class& cl) {
    base_type::extension_def(cl);

    cl.def("size", &base_type::size)
        .def("empty", &empty)
        .def("push_back", &base_type::append);
  }

  static bool empty(Container const& container) { return container.empty(); }
};
}  // namespace pyhpp

#endif  // PYHPP_VECTOR_INDEXING_SUITE_HH
