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

#include <boost/python.hpp>
#include <hpp/pinocchio/fwd.hh>

template<typename T1, typename T2>
  struct PairToPythonConverter {
    static PyObject* convert(const std::pair<T1, T2>& pair)
    {
      return boost::python::incref(boost::python::make_tuple(pair.first, pair.second).ptr());
    }
  };

template<typename T1, typename T2>
  struct PythonToPairConverter {
    PythonToPairConverter()
    {
      boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<std::pair<T1, T2> >());
    }
    static void* convertible(PyObject* obj)
    {
      if (!PyTuple_CheckExact(obj)) return 0;
      if (PyTuple_Size(obj) != 2) return 0;
      return obj;
    }
    static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
      boost::python::tuple tuple(boost::python::borrowed(obj));
      void* storage = ((boost::python::converter::rvalue_from_python_storage<std::pair<T1, T2> >*) data)->storage.bytes;
      new (storage) std::pair<T1, T2>(boost::python::extract<T1>(tuple[0]), boost::python::extract<T2>(tuple[1]));
      data->convertible = storage;
    }
  };

template<typename T1, typename T2>
  struct py_pair {
    boost::python::to_python_converter<std::pair<T1, T2>, PairToPythonConverter<T1, T2> > toPy;
    PythonToPairConverter<T1, T2> fromPy;
  };

BOOST_PYTHON_MODULE(pyhpp)
{
  using namespace hpp::pinocchio;
  py_pair<size_type,size_type>();
  py_pair<value_type,value_type>();
}
