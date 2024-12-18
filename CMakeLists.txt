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
set(PROJECT_URL "https://github.com/humanoid-path-planner/${PROJECT_NAME}")

set(CXX_DISABLE_WERROR TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/test.cmake")

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

add_project_dependency(eigenpy REQUIRED)
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(hpp-util REQUIRED)
add_project_dependency(hpp-pinocchio REQUIRED)
add_project_dependency(hpp-constraints REQUIRED)
add_project_dependency(hpp-core REQUIRED)
add_project_dependency(hpp-corbaserver REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
add_subdirectory(tests)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
