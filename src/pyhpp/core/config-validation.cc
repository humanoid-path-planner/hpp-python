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

#include <pyhpp/core/fwd.hh>

#include <boost/python.hpp>

#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>

#include <pyhpp/util.hh>

using namespace boost::python;

namespace pyhpp {
  namespace core {
    using namespace hpp::core;

    struct CVWrapper {
      static bool validate (ConfigValidation* cv,
          const Configuration_t& config,
          ValidationReportPtr_t& report)
      {
        return cv->validate (config, report);
      }

      static tuple py_validate (ConfigValidation* cv,
          const Configuration_t& config)
      {
        ValidationReportPtr_t report;
        bool res = cv->validate (config, report);
        return boost::python::make_tuple (res, report);
      }
    };

    void exposeConfigValidation ()
    {
      class_ <ConfigValidation, ConfigValidationPtr_t, boost::noncopyable>
        ("ConfigValidation", no_init)
        PYHPP_DEFINE_METHOD (ConfigValidation, validate)

        .def ("validate", &CVWrapper::py_validate)
        ;

      class_ <ConfigValidations, ConfigValidationsPtr_t,
             bases<ConfigValidation>, boost::noncopyable>
        ("ConfigValidations", no_init)
        PYHPP_DEFINE_METHOD (ConfigValidations, add)
        PYHPP_DEFINE_METHOD (ConfigValidations, numberConfigValidations)
        PYHPP_DEFINE_METHOD (ConfigValidations, clear)
        ;

    }
  }
}

