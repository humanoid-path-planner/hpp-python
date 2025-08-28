#
# Copyright (c) 2018 CNRS Authors: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

python_install_on_site(pyhpp/manipulation constraint_graph_factory.py)

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
  pyhpp/constraints/locked-joint.cc
  pyhpp/constraints/bindings.cc
  LINK_LIBRARIES
  hpp-constraints::hpp-constraints)

add_python_library(
  pyhpp/core
  FILES
  pyhpp/core/node.cc
  pyhpp/core/roadmap.cc
  pyhpp/core/config-validation.cc
  pyhpp/core/configuration-shooter.cc
  pyhpp/core/constraint.cc
  pyhpp/core/reports.cc
  pyhpp/core/steering-method.cc
  pyhpp/core/path.cc
  pyhpp/core/path-optimizer.cc
  pyhpp/core/path-projector.cc
  pyhpp/core/path-validation.cc
  pyhpp/core/path-planner.cc
  pyhpp/core/connected-component.cc
  pyhpp/core/distance.cc
  pyhpp/core/problem-target.cc
  pyhpp/core/parameter.cc
  pyhpp/core/problem.cc
  pyhpp/core/problem-solver.cc
  pyhpp/core/bindings.cc
  LINK_LIBRARIES
  hpp-core::hpp-core)

add_python_library(
  pyhpp/core/problem_target FILES
  pyhpp/core/problem_target/goal-configurations.cc
  pyhpp/core/problem_target/bindings.cc LINK_LIBRARIES hpp-core::hpp-core)

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

add_python_library(
  pyhpp/manipulation
  FILES
  pyhpp/manipulation/bindings.cc
  pyhpp/manipulation/device.cc
  pyhpp/manipulation/graph.cc
  pyhpp/manipulation/problem.cc
  pyhpp/manipulation/path-projector.cc
  pyhpp/manipulation/steering-method.cc
  LINK_LIBRARIES
  hpp-manipulation::hpp-manipulation)

add_python_library(
  pyhpp/manipulation/urdf FILES pyhpp/manipulation/urdf/util.cc
  pyhpp/manipulation/urdf/bindings.cc LINK_LIBRARIES
  hpp-manipulation-urdf::hpp-manipulation-urdf)
