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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hpp-python)
SET(PROJECT_DESCRIPTION "Python bindings for HPP.")
SET(PROJECT_URL "")

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()
# Activate hpp-util logging if requested
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmark output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()
# Ask Doxygen to create a tree view in html documentation
SET(DOXYGEN_TREEVIEW "NO" CACHE STRING "Set to YES to generate a tree view in the html documentation")

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
  include/pyhpp/core/fwd.hh
  )

FINDPYTHON(2.7 EXACT REQUIRED)
SET(BOOST_COMPONENTS python)
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")
ADD_REQUIRED_DEPENDENCY("eigenpy >= 1.2")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4")

ADD_SUBDIRECTORY(src)
# ADD_SUBDIRECTORY(tests)

# Add dependency toward hpp-python library in pkg-config file.
# PKG_CONFIG_APPEND_LIBS("hpp-python")

SETUP_PROJECT_FINALIZE()
