#
# Copyright (c) 2018 CNRS Authors: Joseph Mirabel
#
# This file is part of hpp-python hpp-python is free software: you can
# redistribute it and/or modify it under the terms of the GNU Lesser General
# Public License as published by the Free Software Foundation, either version 3
# of the License, or (at your option) any later version.
#
# hpp-python is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU General Lesser Public License for more
# details.  You should have received a copy of the GNU Lesser General Public
# License along with hpp-python  If not, see <http://www.gnu.org/licenses/>.

# ADD_PYTHON_LIBRARY ( module FILES file1.cc file2.cc ... LINK_LIBRARIES
# hpp-core::hpp-core ...)
macro(ADD_PYTHON_LIBRARY MODULE)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs FILES PY_FILES LINK_LIBRARIES)
  cmake_parse_arguments(ADD_PYTHON_LIBRARY "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${MODULE})

  set(GENERATED_FILES)
  foreach(F ${ADD_PYTHON_LIBRARY_FILES})
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${F}
      COMMAND ${CMAKE_SOURCE_DIR}/doc/configure.py
              ${CMAKE_CURRENT_SOURCE_DIR}/${F} ${CMAKE_CURRENT_BINARY_DIR}/${F}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${F}
      COMMENT "Generate with documentation for ${F}")
    list(APPEND GENERATED_FILES ${CMAKE_CURRENT_BINARY_DIR}/${F})
  endforeach()

  string(REGEX REPLACE "[/-]" "_" TARGET_NAME "${MODULE}")
  set(LIBNAME "bindings")
  add_library(${TARGET_NAME} SHARED ${GENERATED_FILES})
  set_target_properties(
    ${TARGET_NAME}
    PROPERTIES PREFIX ""
               LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}
               LIBRARY_OUTPUT_NAME ${LIBNAME})

  target_link_boost_python(${TARGET_NAME} PUBLIC)
  target_link_libraries(
    ${TARGET_NAME} PUBLIC eigenpy::eigenpy ${ADD_PYTHON_LIBRARY_LINK_LIBRARIES})

  install(TARGETS ${TARGET_NAME} DESTINATION ${PYTHON_SITELIB}/${MODULE})

  # Write __init__.py file
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
       "from .${LIBNAME} import *")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/__init__.py
          DESTINATION ${PYTHON_SITELIB}/${MODULE})
endmacro()

python_install_on_site(pyhpp __init__.py)

add_python_library(
  pyhpp/pinocchio
  FILES
  pyhpp/pinocchio/device.cc
  pyhpp/pinocchio/liegroup.cc
  pyhpp/pinocchio/bindings.cc
  LINK_LIBRARIES
  hpp-pinocchio::hpp-pinocchio)

add_python_library(
  pyhpp/pinocchio/urdf FILES pyhpp/pinocchio/urdf/util.cc
  pyhpp/pinocchio/urdf/bindings.cc LINK_LIBRARIES hpp-pinocchio::hpp-pinocchio)

add_python_library(
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
  LINK_LIBRARIES
  hpp-constraints::hpp-constraints)

add_python_library(
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
  pyhpp/core/parameter.cc
  pyhpp/core/problem.cc
  pyhpp/core/problem-solver.cc
  pyhpp/core/bindings.cc
  LINK_LIBRARIES
  hpp-core::hpp-core)

add_python_library(
  pyhpp/core/path
  FILES
  pyhpp/core/path/spline.cc
  pyhpp/core/path/vector.cc
  pyhpp/core/path/bindings.cc
  LINK_LIBRARIES
  hpp-core::hpp-core)

add_python_library(
  pyhpp/core/path_optimization
  FILES
  pyhpp/core/path_optimization/spline-gradient-based-abstract.cc
  pyhpp/core/path_optimization/bindings.cc
  LINK_LIBRARIES
  hpp-core::hpp-core
  hpp-core::hpp-core-gpl)

add_python_library(
  pyhpp/corbaserver FILES pyhpp/corbaserver/bindings.cc
  pyhpp/corbaserver/server.cc LINK_LIBRARIES hpp-corbaserver::hpp-corbaserver)
