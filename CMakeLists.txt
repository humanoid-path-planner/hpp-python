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

cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME hpp-python)
set(PROJECT_DESCRIPTION "python bindings for HPP, based on boost python")
set(PROJECT_URL "")

set(CXX_DISABLE_WERROR TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)
include(cmake/python.cmake)
include(cmake/boost.cmake)
include(cmake/test.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 REQUIRED)

# Activate hpp-util logging if requested
set(HPP_DEBUG
    FALSE
    CACHE BOOL "trigger hpp-util debug output")
if(HPP_DEBUG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
endif()
# Activate hpp-util logging if requested
set(HPP_BENCHMARK
    FALSE
    CACHE BOOL "trigger hpp-util benchmark output")
if(HPP_BENCHMARK)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
endif()
# Ask Doxygen to create a tree view in html documentation
set(DOXYGEN_TREEVIEW
    "NO"
    CACHE STRING "Set to YES to generate a tree view in the html documentation")

# Declare Headers
set(${PROJECT_NAME}_HEADERS include/pyhpp/core/fwd.hh)

set(PYTHON_COMPONENTS Interpreter Development NumPy)
findpython()
search_for_boost_python()

add_project_dependency(eigenpy 3 REQUIRED)
add_project_dependency(pinocchio 2.6 REQUIRED)
add_project_dependency(hpp-util 4 REQUIRED)
add_project_dependency(hpp-pinocchio 4 REQUIRED)
add_project_dependency(hpp-constraints 4 REQUIRED)
add_project_dependency(hpp-core 4 REQUIRED)
add_project_dependency(hpp-corbaserver 4 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
add_subdirectory(tests)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
