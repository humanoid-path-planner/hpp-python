#
# Copyright (c) 2018 CNRS
# Authors: Joseph Mirabel
#
#
# This file is part of hpp-python
# hpp-python is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-python is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-python  If not, see
# <http://www.gnu.org/licenses/>.

# ADD_PYTHON_LIBRARY ( libname module FILES file1.cc file2.cc)
MACRO(ADD_PYTHON_LIBRARY LIBNAME MODULE)
  CMAKE_PARSE_ARGUMENTS(ADD_PYTHON_LIBRARY "" "" "FILES" ${ARGN})
  ADD_LIBRARY(${LIBNAME} SHARED ${ADD_PYTHON_LIBRARY_FILES})
  SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}
    )

  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" ${PYTHON_INCLUDE_DIRS})
  TARGET_LINK_BOOST_PYTHON(${LIBNAME})
  PKG_CONFIG_USE_DEPENDENCY (${LIBNAME} "hpp-core")
  PKG_CONFIG_USE_DEPENDENCY (${LIBNAME} "eigenpy")
  TARGET_LINK_LIBRARIES(${LIBNAME} ${Boost_LIBRARIES})

  INSTALL(TARGETS ${LIBNAME}
    DESTINATION ${PYTHON_SITELIB}/${MODULE})

  # Write __init__.py file
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
    "from ${LIBNAME} import *")
    # "from lib${LIBNAME} import *")
  INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${MODULE})
ENDMACRO()

PYTHON_INSTALL_ON_SITE(pyhpp __init__.py)

ADD_PYTHON_LIBRARY(
  pyhpppinocchio
  pyhpp/pinocchio
  FILES
  pyhpp/pinocchio/device.cc
  pyhpp/pinocchio/liegroup.cc
  pyhpp/pinocchio/bindings.cc
  )

ADD_PYTHON_LIBRARY(
  pyhpppinocchiourdf
  pyhpp/pinocchio/urdf
  FILES
  pyhpp/pinocchio/urdf/util.cc
  pyhpp/pinocchio/urdf/bindings.cc
  )

ADD_PYTHON_LIBRARY(
  pyhppconstraints
  pyhpp/constraints
  FILES
  pyhpp/constraints/differentiable-function.cc
  pyhpp/constraints/explicit-solver.cc
  pyhpp/constraints/generic-transformation.cc
  pyhpp/constraints/iterative-solver.cc
  pyhpp/constraints/hybrid-solver.cc
  pyhpp/constraints/bindings.cc
  )

ADD_PYTHON_LIBRARY(
  pyhppcore
  pyhpp/core
  FILES
  pyhpp/core/path.cc
  pyhpp/core/path-optimizer.cc
  pyhpp/core/problem-solver.cc
  pyhpp/core/bindings.cc
  )

ADD_PYTHON_LIBRARY(
  pyhppcorepath
  pyhpp/core/path
  FILES
  pyhpp/core/path/spline.cc
  pyhpp/core/path/vector.cc
  pyhpp/core/path/bindings.cc
  )

ADD_PYTHON_LIBRARY(
  pyhppcorepathoptimization
  pyhpp/core/pathoptimization
  FILES
  pyhpp/core/path-optimization/spline-gradient-based-abstract.cc
  pyhpp/core/path-optimization/bindings.cc
  )
