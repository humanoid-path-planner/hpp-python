# Expanded from @PACKAGE_INIT@ by configure_package_config_file() ####### Any
# changes to this file will be overwritten by the next CMake run #### The input
# file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../"
                       ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(
      FATAL_ERROR
        "File or directory ${_file} referenced by variable ${_var} does not exist !"
    )
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

# ##############################################################################

set(skip_this_file TRUE)
if(NOT hpp-python_FOUND)
  set(skip_this_file FALSE)
endif()
if(skip_this_file)
  foreach(component ${hpp-python_FIND_COMPONENTS})
    if(NOT "hpp-python_${component}_FOUND")
      set(skip_this_file FALSE)
    endif()
  endforeach()
endif()
if(skip_this_file)
  return()
endif()

set("hpp-python_INCLUDE_DIRS" "${PACKAGE_PREFIX_DIR}/include")
set("HPP_PYTHON_INCLUDE_DIRS" "${PACKAGE_PREFIX_DIR}/include")
set("hpp-python_DOXYGENDOCDIR"
    "${PACKAGE_PREFIX_DIR}/share/doc/hpp-python/doxygen-html")
set("HPP_PYTHON_DOXYGENDOCDIR"
    "${PACKAGE_PREFIX_DIR}/share/doc/hpp-python/doxygen-html")
set("hpp-python_DEPENDENCIES"
    "eigenpy;pinocchio;hpp-util;hpp-pinocchio;hpp-constraints;hpp-core;hpp-corbaserver;hpp-manipulation;hpp-manipulation-urdf"
)
set("hpp-python_PKG_CONFIG_DEPENDENCIES"
    "omniORB4;omniORB4 >= 4.1.4;omniDynamic4 >= 4.1.4")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

# Find absolute library paths for all _PKG_CONFIG_LIBS as CMake expects full
# paths, while pkg-config does not.
set(_PACKAGE_CONFIG_LIBRARIES "")
set("_hpp-python_PKG_CONFIG_LIBDIR" "${pcfiledir}/../../lib")
set("_hpp-python_PKG_CONFIG_LIBS_LIST" "")
if(_hpp-python_PKG_CONFIG_LIBS_LIST)
  string(FIND ${_hpp-python_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  while(_is_comma_space GREATER -1)
    string(REPLACE ", " "," _hpp-python_PKG_CONFIG_LIBS_LIST
                   "${_hpp-python_PKG_CONFIG_LIBS_LIST}")
    string(FIND ${_hpp-python_PKG_CONFIG_LIBS_LIST} ", " _is_comma_space)
  endwhile()
  string(REPLACE " " ";" _hpp-python_PKG_CONFIG_LIBS_LIST
                 "${_hpp-python_PKG_CONFIG_LIBS_LIST}")
  set(LIBDIR_HINTS ${_hpp-python_PKG_CONFIG_LIBDIR})
  foreach(component ${_hpp-python_PKG_CONFIG_LIBS_LIST})
    string(STRIP ${component} component)
    # If the component is a link directory ("-L/full/path"), append to
    # LIBDIR_HINTS.
    string(FIND ${component} "-L" _is_library_dir)
    if(${_is_library_dir} EQUAL 0)
      string(REGEX REPLACE "^-L" "" lib_path ${component})
      list(APPEND LIBDIR_HINTS "${lib_path}")
      continue()
    endif()
    # If the component is a library name
    string(FIND ${component} "-l" _is_library_name)
    if(${_is_library_name} EQUAL 0)
      string(REGEX REPLACE "^-l" "" lib ${component})
      find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
      if(NOT abs_lib_${lib})
        if(_LIBDIR_HINTS)
          message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
        else()
          message(STATUS "${lib} not FOUND.")
        endif()
      else()
        if(_LIBDIR_HINTS)
          message(
            STATUS
              "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}"
          )
        else()
          message(STATUS "${lib} FOUND. ${lib} at ${abs_lib_${lib}}")
        endif()
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
      endif()
      unset(abs_lib_${lib} CACHE)
      continue()
    endif()
    # If the component contains a collection of additional arguments
    string(FIND ${component} "," _is_collection)
    if(${_is_collection} GREATER -1)
      string(REPLACE "," ";" component_list "${component}")
      list(GET component_list -1 lib_info)
      set(options ${component})
      list(REMOVE_AT options -1)
      string(FIND ${lib_info} "-l" _is_library_name)
      if(${_is_library_name} GREATER -1)
        string(REGEX REPLACE "^-l" "" lib ${lib_info})
        find_library(abs_lib_${lib} ${lib} HINTS ${LIBDIR_HINTS})
        if(NOT abs_lib_${lib})
          if(_LIBDIR_HINTS)
            message(STATUS "${lib} searched on ${_LIBDIR_HINTS} not FOUND.")
          else()
            message(STATUS "${lib} not FOUND.")
          endif()
        else()
          if(_LIBDIR_HINTS)
            message(
              STATUS
                "${lib} searched on ${_LIBDIR_HINTS} FOUND. ${lib} at ${abs_lib_${lib}}"
            )
          else()
            message(STATUS "${lib} FOUND. ${lib} at ${abs_lib_${lib}}")
          endif()
          list(APPEND _PACKAGE_CONFIG_LIBRARIES "${abs_lib_${lib}}")
        endif()
        unset(abs_lib_${lib} CACHE)
        continue()
      else() # This is an absolute lib
        list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
      endif()
      continue()
    endif()
    # Else, this is just an absolute lib
    if(EXISTS "${component}")
      list(APPEND _PACKAGE_CONFIG_LIBRARIES "${component}")
    endif()
  endforeach()
endif(_hpp-python_PKG_CONFIG_LIBS_LIST)

set("hpp-python_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})
set("HPP_PYTHON_LIBRARIES" ${_PACKAGE_CONFIG_LIBRARIES})

include("${CMAKE_CURRENT_LIST_DIR}/cxx-standard.cmake")

include(CMakeFindDependencyMacro)
if(${CMAKE_VERSION} VERSION_LESS "3.15.0")
  find_package(eigenpy REQUIRED)
  find_package(pinocchio REQUIRED)
  find_package(hpp-util REQUIRED)
  find_package(hpp-pinocchio REQUIRED)
  find_package(hpp-constraints REQUIRED)
  find_package(hpp-core REQUIRED)
  find_package(hpp-corbaserver REQUIRED)
  find_package(hpp-manipulation REQUIRED)
  find_package(hpp-manipulation-urdf REQUIRED)
else()
  find_dependency(eigenpy REQUIRED)
  find_dependency(pinocchio REQUIRED)
  find_dependency(hpp-util REQUIRED)
  find_dependency(hpp-pinocchio REQUIRED)
  find_dependency(hpp-constraints REQUIRED)
  find_dependency(hpp-core REQUIRED)
  find_dependency(hpp-corbaserver REQUIRED)
  find_dependency(hpp-manipulation REQUIRED)
  find_dependency(hpp-manipulation-urdf REQUIRED)
endif()

if(COMMAND ADD_REQUIRED_DEPENDENCY)
  foreach(pkgcfg_dep ${hpp-python_PKG_CONFIG_DEPENDENCIES})
    # Avoid duplicated lookup.
    list(FIND $_PKG_CONFIG_REQUIRES "${pkgcfg_dep}" _index)
    if(${_index} EQUAL -1)
      add_required_dependency(${pkgcfg_dep})
    endif()
  endforeach()
endif(COMMAND ADD_REQUIRED_DEPENDENCY)

include("${CMAKE_CURRENT_LIST_DIR}/hpp-pythonTargets.cmake")

foreach(component ${hpp-python_FIND_COMPONENTS})
  set(comp_file "${CMAKE_CURRENT_LIST_DIR}/${component}Config.cmake")
  if(EXISTS ${comp_file})
    include(${comp_file})
  else()
    set(hpp-python_${component}_FOUND FALSE)
  endif()
  if(hpp-python_${component}_FOUND)
    message(STATUS "hpp-python: ${component} found.")
  else()
    if(hpp-python_FIND_REQUIRED_${component})
      message(FATAL_ERROR "hpp-python: ${component} not found.")
    else()
      message(STATUS "hpp-python: ${component} not found.")
    endif()
  endif()
endforeach()
check_required_components("hpp-python")

check_minimal_cxx_standard(14)
