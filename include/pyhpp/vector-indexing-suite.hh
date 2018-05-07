//
// Copyright (c) 2018 CNRS
// Authors: Joseph Mirabel
//
//
// This file is part of hpp-python
// hpp-python is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef PYHPP_VECTOR_INDEXING_SUITE_HH
#define PYHPP_VECTOR_INDEXING_SUITE_HH

# include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace pyhpp {
  template <class Container,
           bool NoProxy = false>
  class cpp_like_vector_indexing_suite
  : public boost::python::vector_indexing_suite <Container, NoProxy, cpp_like_vector_indexing_suite<Container, NoProxy> >
  {
    public:
      typedef boost::python::vector_indexing_suite <Container, NoProxy, cpp_like_vector_indexing_suite> base_type;
      template <class Class>
        static void 
        extension_def(Class& cl)
        {
          base_type::extension_def (cl);

          cl
            .def("size", &base_type::size)
            .def("empty", &empty)
            .def("push_back", &base_type::append)
            ;
        }

        static bool 
        empty(Container const& container)
        { 
            return container.empty();
        }
  };
}

#endif // PYHPP_VECTOR_INDEXING_SUITE_HH
