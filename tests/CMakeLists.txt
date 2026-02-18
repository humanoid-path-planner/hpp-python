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

# =============================================================================
# unittest-based unit tests (uses Python's built-in unittest framework)
# =============================================================================

# Compute Python path for unit tests
compute_pythonpath(UNITTEST_ENV_VARIABLES src)

# Discover and add individual test files
file(GLOB UNITTEST_TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/unit/test_*.py")
foreach(TEST_FILE ${UNITTEST_TEST_FILES})
  get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
  add_test(
    NAME unittest_${TEST_NAME}
    COMMAND ${PYTHON_EXECUTABLE} -m unittest discover -s
            ${CMAKE_CURRENT_SOURCE_DIR} -p "${TEST_NAME}.py" -v
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  set_tests_properties(
    unittest_${TEST_NAME}
    PROPERTIES
      ENVIRONMENT
      "${UNITTEST_ENV_VARIABLES}"
      ENVIRONMENT_MODIFICATION
      "ROS_PACKAGE_PATH=path_list_prepend:${example-robot-data_INCLUDE_DIRS}/../share;ROS_PACKAGE_PATH=path_list_prepend:${hpp-environments_INCLUDE_DIRS}/../share;"
      LABELS
      "python;unit")
endforeach()
