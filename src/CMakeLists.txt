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

# ADD_PYTHON_LIBRARY ( module
#   FILES file1.cc file2.cc ...
#   PKG_CONFIG_DEPENDENCIES hpp-core ...)
MACRO(ADD_PYTHON_LIBRARY MODULE)
  set(options )
  set(oneValueArgs )
  SET(multiValueArgs FILES PKG_CONFIG_DEPENDENCIES ADDITIONAL_LIBS)
  CMAKE_PARSE_ARGUMENTS(ADD_PYTHON_LIBRARY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${MODULE})

  SET(GENERATED_FILES)
  FOREACH(F ${ADD_PYTHON_LIBRARY_FILES})
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${F}
        COMMAND ${CMAKE_SOURCE_DIR}/doc/configure.py
        ${CMAKE_CURRENT_SOURCE_DIR}/${F}
        ${CMAKE_CURRENT_BINARY_DIR}/${F}
        DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${F}
        COMMENT "Generate with documentation for ${F}")
    LIST(APPEND GENERATED_FILES ${CMAKE_CURRENT_BINARY_DIR}/${F})
  ENDFOREACH()

  STRING(REGEX REPLACE "[/-]" "_" TARGET_NAME "${MODULE}")
  SET(LIBNAME "bindings")
  ADD_LIBRARY(${TARGET_NAME} SHARED ${GENERATED_FILES})
  SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}
    LIBRARY_OUTPUT_NAME ${LIBNAME}
    )

  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" ${PYTHON_INCLUDE_DIRS})
  TARGET_LINK_BOOST_PYTHON(${TARGET_NAME})
  FOREACH (PKG_DEPENDENCY ${ADD_PYTHON_LIBRARY_PKG_CONFIG_DEPENDENCIES})
    PKG_CONFIG_USE_DEPENDENCY (${TARGET_NAME} ${PKG_DEPENDENCY})
  ENDFOREACH ()
  FOREACH(LIB ${ADD_PYTHON_LIBRARY_ADDITIONAL_LIBS})
    MESSAGE(STATUS "additional lib: ${LIB}")
    TARGET_LINK_LIBRARIES(${TARGET_NAME} ${LIB})
  ENDFOREACH()
  TARGET_LINK_LIBRARIES(${TARGET_NAME} ${Boost_LIBRARIES})

  INSTALL(TARGETS ${TARGET_NAME}
    DESTINATION ${PYTHON_SITELIB}/${MODULE})

  # Write __init__.py file
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
    "from .${LIBNAME} import *")
  INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${MODULE})
ENDMACRO()

LIST(APPEND DEPENDENCIES "eigenpy" "pinocchio" "hpp-util" "hpp-pinocchio")

PYTHON_INSTALL_ON_SITE(pyhpp __init__.py)

ADD_PYTHON_LIBRARY(
  pyhpp/pinocchio
  FILES
  pyhpp/pinocchio/device.cc
  pyhpp/pinocchio/liegroup.cc
  pyhpp/pinocchio/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )

ADD_PYTHON_LIBRARY(
  pyhpp/pinocchio/urdf
  FILES
  pyhpp/pinocchio/urdf/util.cc
  pyhpp/pinocchio/urdf/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )

LIST(APPEND DEPENDENCIES "hpp-constraints")
ADD_PYTHON_LIBRARY(
  pyhpp/constraints
  FILES
  pyhpp/constraints/differentiable-function.cc
  pyhpp/constraints/explicit.cc
  pyhpp/constraints/explicit-constraint-set.cc
  pyhpp/constraints/generic-transformation.cc
  pyhpp/constraints/implicit.cc
  pyhpp/constraints/iterative-solver.cc
  pyhpp/constraints/by-substitution.cc
  pyhpp/constraints/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )

LIST(APPEND DEPENDENCIES "hpp-core")
ADD_PYTHON_LIBRARY(
  pyhpp/core
  FILES
  pyhpp/core/config-validation.cc
  pyhpp/core/configuration-shooter.cc
  pyhpp/core/constraint.cc
  pyhpp/core/reports.cc
  pyhpp/core/steering-method.cc
  pyhpp/core/path.cc
  pyhpp/core/path-optimizer.cc
  pyhpp/core/path-projector.cc
  pyhpp/core/path-validation.cc
  pyhpp/core/problem.cc
  pyhpp/core/problem-solver.cc
  pyhpp/core/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )

ADD_PYTHON_LIBRARY(
  pyhpp/core/path
  FILES
  pyhpp/core/path/spline.cc
  pyhpp/core/path/vector.cc
  pyhpp/core/path/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )

ADD_PYTHON_LIBRARY(
  pyhpp/core/path_optimization
  FILES
  pyhpp/core/path_optimization/spline-gradient-based-abstract.cc
  pyhpp/core/path_optimization/bindings.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  ADDITIONAL_LIBS hpp-core-gpl
  )

LIST(APPEND DEPENDENCIES "hpp-corbaserver")
ADD_PYTHON_LIBRARY(
  pyhpp/corbaserver
  FILES
  pyhpp/corbaserver/bindings.cc
  pyhpp/corbaserver/server.cc
  PKG_CONFIG_DEPENDENCIES ${DEPENDENCIES}
  )
