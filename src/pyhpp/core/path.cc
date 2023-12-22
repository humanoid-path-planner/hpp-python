//
// Copyright (c) 2018 - 2023, CNRS
// Authors: Joseph Mirabel, Florent Lamiraux
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

#include <hpp/core/path.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/python/config.hh>
#include <pyhpp/core/fwd.hh>
#include <pyhpp/util.hh>

#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>

using namespace boost::python;

namespace pyhpp {
namespace core {
using namespace hpp::core;

class PathWrapper : public Path {
  public:
    virtual ~PathWrapper() = default;
  protected:
    using Path::Path;
};

struct PathWrap : PathWrapper, wrapper<PathWrapper> {
  PathWrap(const interval_t& interval, size_type outputSize,
       size_type outputDerivativeSize, const ConstraintSetPtr_t& constraints)
    : PathWrapper(interval, outputSize, outputDerivativeSize, constraints) {}

  PathWrap(const interval_t& interval, size_type outputSize,
       size_type outputDerivativeSize)
    : PathWrapper(interval, outputSize, outputDerivativeSize) {}

  std::ostream& print(std::ostream& os) const {
    return Path::print(os << "PathWrap: ");
  }

  void init(hpp::shared_ptr<PathWrap>& shPtr) {
    Path::init(shPtr);
  }

  boost::python::override get_override_or_throw(const char* funcname) const {
    boost::python::override func = this->get_override(funcname);
    if (func.is_none()) {
      std::ostringstream oss;
      oss << "Class inheriting from " "PathWrap" " must implement " << funcname;
      throw std::runtime_error(oss.str());
    }
    return func;
  }


  PathPtr_t copy() const override { return get_override_or_throw("copy")(); }
  PathPtr_t copy_constrained(const ConstraintSetPtr_t& constraints) const {
    return get_override_or_throw("copy_constrained")(constraints);
  }
  PathPtr_t copy(const ConstraintSetPtr_t& constraints) const override { return copy_constrained(constraints); }

  PathPtr_t reverse() const override {
    if (override reverse = this->get_override("reverse"))
      return reverse();
    return Path::reverse();
  }
  PathPtr_t default_reverse() const { return this->Path::reverse(); }

  Configuration_t initial() const override { return get_override_or_throw("initial")(); }
  Configuration_t end() const override { return get_override_or_throw("end")(); }

  bool impl_compute(ConfigurationOut_t configuration, value_type param) const override {
    object obj = get_override_or_throw("impl_compute")(param);
    using boost::python::extract; // As Path::extract exists.
    tuple tup = extract<tuple>(obj);
    configuration = extract<vector_t>(tup[0])();
    bool ok = extract<bool>(tup[1]);
    return ok;
  }

  void impl_derivative(vectorOut_t derivative, const value_type& param, size_type order) const override {
    object obj = get_override_or_throw("impl_derivative")(param, order);
    using boost::python::extract; // As Path::extract exists.
    derivative = extract<vector_t>(obj)();
  }

  static tuple py_call1(const Path& p, const value_type& t) {
    bool success;
    Configuration_t q(p.eval(t, success));
    return make_tuple(q, success);
  }
  static bool py_call2(const Path& p, ConfigurationOut_t q,
                       const value_type& t) {
    Configuration_t qq(q);
    bool success = p.eval(qq, t);
    q = qq;
    return success;
  }
  static vector_t derivative(const Path& p, const value_type& t, size_type order) {
    vector_t d(p.outputDerivativeSize());
    p.derivative(d, t, order);
    return d;
  }
  static ConstraintSetPtr_t constraints(const Path& p) {
    return p.constraints();
  }
};

void exposePath() {
  class_<Path, hpp::shared_ptr<Path>, boost::noncopyable>("Path", no_init)
    .def("__str__", &to_str_from_operator<Path>)

    .def("__call__", &PathWrap::py_call1)
    .def("__call__", &PathWrap::py_call2)
    .def("eval", &PathWrap::py_call1)
    .def("eval", &PathWrap::py_call2)
    .def("derivative", &PathWrap::derivative)

    .def("copy", static_cast<PathPtr_t (Path::*)() const>(&Path::copy))
    .def("extract" ,static_cast<PathPtr_t (Path::*)(const value_type&,
         const value_type&) const>(&Path::extract))
    .def("extract" ,static_cast<PathPtr_t (Path::*)(const interval_t&) const>(&Path::extract))
    // .PYHPP_DEFINE_METHOD (Path, timeRange)
    .def("timeRange",
        static_cast<const interval_t& (Path::*)() const>(&Path::timeRange),
        return_internal_reference<>())
    .PYHPP_DEFINE_METHOD_INTERNAL_REF(Path, paramRange)
    .PYHPP_DEFINE_METHOD(Path, length)
    .PYHPP_DEFINE_METHOD(Path, initial)
    .PYHPP_DEFINE_METHOD(Path, end)
    .PYHPP_DEFINE_METHOD(PathWrap, constraints)
  ;

  class_<PathWrap, bases<Path>, hpp::shared_ptr<PathWrap>, boost::noncopyable>("PathWrap", no_init)
    .def(init<interval_t, size_type, size_type>())
    .def("initPtr", &PathWrap::init)

    .def("initial", pure_virtual(&PathWrap::initial))
    .def("end", pure_virtual(&PathWrap::end))
    .def("impl_compute", pure_virtual(&PathWrap::impl_compute))
    .def("impl_derivative", pure_virtual(&PathWrap::impl_derivative))

    .def("copy", pure_virtual(static_cast<PathPtr_t (PathWrap::*)() const>(&PathWrap::copy)))
    .def("copy_constrained", pure_virtual(&PathWrap::copy_constrained))

    .def("reverse", &Path::reverse, &PathWrap::default_reverse)
  ;

  def("create", +[](double d, int ndof) -> hpp::shared_ptr<PathWrap> {
        hpp::shared_ptr<PathWrap> ptr(new PathWrap(interval_t(0,d), ndof, ndof));
        ptr->init(ptr);
        return ptr;
        });

  class_<StraightPath, bases<Path>, StraightPathPtr_t, boost::noncopyable>("StraightPath", no_init)
    .def("create", static_cast<
        StraightPathPtr_t (*) (LiegroupSpacePtr_t, vectorIn_t, vectorIn_t,
          interval_t, ConstraintSetPtr_t)>(&StraightPath::create))
    .def("create", static_cast<
        StraightPathPtr_t (*) (const DevicePtr_t&, ConfigurationIn_t, ConfigurationIn_t,
          interval_t, ConstraintSetPtr_t)>(&StraightPath::create))
    .staticmethod("create")
  ;

}
}  // namespace core
}  // namespace pyhpp
