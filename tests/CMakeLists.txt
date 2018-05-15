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

SET(PYTHON_UNIT_TESTS
  imports
  liegroup
  load_ur3
  solver
  differentiable-function
  problem-solver
  )

FOREACH(UNIT_TEST ${PYTHON_UNIT_TESTS})
  ADD_PYTHON_UNIT_TEST(${UNIT_TEST} ${CMAKE_CURRENT_SOURCE_DIR}/${UNIT_TEST}.py)
ENDFOREACH()

SET_TESTS_PROPERTIES(${PYTHON_UNIT_TESTS}
  PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/src:$ENV{PYTHONPATH}")
