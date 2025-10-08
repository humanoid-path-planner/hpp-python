# Install script for directory: /home/psardin/devel/nix-hpp/src/hpp-python/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/psardin/devel/nix-hpp/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" "" CMAKE_INSTALL_CONFIG_NAME
                         "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP
      "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/objdump"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp"
    TYPE FILE FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/src/pyhpp/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/src/pyhpp/manipulation/constraint_graph_factory.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/pinocchio/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_pinocchio.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/pinocchio/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/pinocchio/urdf/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_pinocchio_urdf.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/pinocchio/urdf"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/pinocchio/urdf/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/constraints/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_constraints.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/constraints"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/constraints/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_core.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/problem_target/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_core_problem_target.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/problem_target"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/problem_target/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/path/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_core_path.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/path/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/path_optimization/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_core_path_optimization.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/core/path_optimization"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/core/path_optimization/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/corbaserver/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_corbaserver.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/corbaserver"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/corbaserver/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/manipulation/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_manipulation.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/manipulation/__init__.py"
  )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
  )
    file(
      RPATH_CHECK
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
      RPATH
      "")
  endif()
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf"
    TYPE
    SHARED_LIBRARY
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/manipulation/urdf/bindings.so"
  )
  if(EXISTS
     "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
     AND NOT
         IS_SYMLINK
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
  )
    file(
      RPATH_CHANGE
      FILE
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
      OLD_RPATH
      "/home/psardin/devel/nix-hpp/install/lib:"
      NEW_RPATH
      "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(
        COMMAND
          "/nix/store/8v6k283dpbc0qkdq81nb6mrxrgcb10i1-gcc-wrapper-14-20241116/bin/strip"
          "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf/bindings.so"
      )
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)

endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  include(
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/CMakeFiles/pyhpp_manipulation_urdf.dir/install-cxx-module-bmi-Release.cmake"
    OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT
                                                     CMAKE_INSTALL_COMPONENT)
  file(
    INSTALL
    DESTINATION
      "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/pyhpp/manipulation/urdf"
    TYPE
    FILE
    FILES
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/pyhpp/manipulation/urdf/__init__.py"
  )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
               "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(
    WRITE
    "/home/psardin/devel/nix-hpp/src/hpp-python/build-rel/src/install_local_manifest.txt"
    "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
