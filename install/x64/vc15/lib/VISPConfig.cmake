#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# CMake package config file for ViSP.
#
# ** File generated automatically, do not modify **
#
# This file will define the following CMake variables:
#   - VISP_LIBS           : The list of all imported targets for VISP modules.
#   - VISP_INCLUDE_DIRS   : ViSP and third-party include directories
#   - VISP_LIBRARIES      : ViSP library to link against. Third-party libraries are
#                           linked automatically thanks to cmake export file VISPModules.cmake
#   - VISP_VERSION_STRING : Full ViSP version that is build, "3.2.1"
#   - VISP_VERSION_MAJOR  : Major version part of VISP_VERSION: "3"
#   - VISP_VERSION_MINOR  : Minor version part of VISP_VERSION: "2"
#   - VISP_VERSION_PATCH  : Patch version part of VISP_VERSION: "1"
#
#  Advanced variables:
#    - VISP_SHARED        : Use ViSP as shared library
#    - VISP_CONFIG_PATH   : Path to this VISPConfig.cmake
#    - VISP_USE_FILE      : File to include to use ViSP without specific cmake code
#    - VISP_LIB_COMPONENTS: Present VISP modules list
#
# Typical usage in user project:
#
#   find_package(VISP REQUIRED)
#   include_directories(${VISP_INCLUDE_DIRS})
#   target_link_libraries(MY_TARGET_NAME ${VISP_LIBRARIES})
#
# It is also possible to build your project using VISP_USE_FILE.
#
#   find_package(VISP)
#   if(VISP_FOUND)
#     include(${VISP_USE_FILE})
#   endif()
#
# Or you can search for specific ViSP modules:
#
#   find_package(VISP REQUIRED core io)
#
# If the module is found then VISP_<MODULE>_FOUND is set to TRUE.
#
# Authors:
# Fabien Spindler
#
#############################################################################

#if(VISP_FOUND)
#  return()
#endif()

#set(VISP_FOUND TRUE)

# Set the version numbers
set(VISP_VERSION_STRING "3.2.1")
set(VISP_VERSION_MAJOR  "3")
set(VISP_VERSION_MINOR  "2")
set(VISP_VERSION_PATCH  "1")

set(VISP_LIB_COMPONENTS visp_vs;visp_visual_features;visp_vision;visp_tt_mi;visp_tt;visp_me;visp_mbt;visp_klt;visp_blob;visp_sensor;visp_robot;visp_io;visp_imgproc;visp_gui;visp_detection;visp_core;visp_ar)

# Form list of modules (components) to find
if(NOT VISP_FIND_COMPONENTS)
  set(VISP_FIND_COMPONENTS ${VISP_LIB_COMPONENTS})
endif()

# expand short module names and see if requested components exist
set(VISP_FIND_COMPONENTS_ "")
foreach(__vpcomponent ${VISP_FIND_COMPONENTS})
  if(NOT __vpcomponent MATCHES "^visp_")
    set(__vpcomponent visp_${__vpcomponent})
  endif()
  list(FIND VISP_LIB_COMPONENTS ${__vpcomponent} __vpcomponentIdx)
  if(__vpcomponentIdx LESS 0)
    #requested component is not found...
    if(VISP_FIND_REQUIRED)
      message(FATAL_ERROR "${__vpcomponent} is required but was not found")
    elseif(NOT VISP_FIND_QUIETLY)
      message(WARNING "${__vpcomponent} is required but was not found")
    endif()
    #indicate that module is NOT found
    string(TOUPPER "${__vpcomponent}" __vpcomponentUP)
    set(${__vpcomponentUP}_FOUND "${__vpcomponentUP}_FOUND-NOTFOUND")
  else()
    list(APPEND VISP_FIND_COMPONENTS_ ${__vpcomponent})
    # Not using list(APPEND) here, because VISP_LIBS may not exist yet.
    # Also not clearing VISP_LIBS anywhere, so that multiple calls
    # to find_package(VISP) with different component lists add up.
    set(VISP_LIBS ${VISP_LIBS} "${__vpcomponent}")
    #indicate that module is found
    string(TOUPPER "${__vpcomponent}" __vpcomponentUP)
    set(${__vpcomponentUP}_FOUND 1)
  endif()
endforeach()
set(VISP_FIND_COMPONENTS ${VISP_FIND_COMPONENTS_})
#message("VISP_FIND_COMPONENTS: ${VISP_FIND_COMPONENTS}")

# Resolve dependencies
foreach(__opttype OPT DBG)
  set(VISP_LIBS_${__opttype} "${VISP_LIBS}")
endforeach()

# Some additional settings are required if ViSP is built as static libs
set(VISP_SHARED ON)

# Extract the directory where *this* file has been installed (determined at cmake run-time)
set(VISP_CONFIG_PATH ${VISP_DIR} CACHE PATH "ViSPConfig.cmake path location" FORCE)
mark_as_advanced(VISP_CONFIG_PATH)

if(NOT WIN32)
  # Since this file is installed in ./${VISP_INSTALL_LIBDIR}/cmake/visp
  # the first "../../" are here to go to ${VISP_INSTALL_LIBDIR}, and then
  # VISP_INSTALL_LIBDIR_TO_PARENT goes to the parent dir "."
  set(VISP_INSTALL_PATH "${VISP_CONFIG_PATH}/../../../../../")
  # Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
  if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_LESS 2.8)
    get_filename_component(VISP_INSTALL_PATH "${VISP_INSTALL_PATH}" ABSOLUTE)
  else()
    get_filename_component(VISP_INSTALL_PATH "${VISP_INSTALL_PATH}" REALPATH)
  endif()
endif()

# Tells the user project where to find ViSP headers
set(VISP_INCLUDE_DIRS "${VISP_CONFIG_PATH}/include;D:/visp-ws/opencv-4.1.1/build/include;D:/visp-ws/eigen-3.3.7/build-vc15/install/include/eigen3;C:/Program Files/Basler/pylon 5/Development/include;C:/Program Files (x86)/Intel RealSense SDK 2.0 (Win7)/include")

# Tells the user project ViSP library name
set(VISP_LIBRARIES ${VISP_LIBS})

# need to be improved
if(POLICY CMP0024)
  # Fix to prevent multiple includes
  if(NOT TARGET visp_core)
    cmake_policy(PUSH)
    cmake_policy(SET CMP0024 NEW)
    # Our library dependencies (contains definitions for IMPORTED targets)
    include("${CMAKE_CURRENT_LIST_DIR}/VISPModules.cmake")
    cmake_policy(POP)
  endif()
else()
  # Fix for cmake 2.8.7 to prevent multiple includes
  if(NOT TARGET visp_core)
    # Our library dependencies (contains definitions for IMPORTED targets)
    include("${CMAKE_CURRENT_LIST_DIR}/VISPModules.cmake")
  endif()
endif()

# where to find the USE file to be used by user project
set(VISP_USE_FILE "${CMAKE_CURRENT_LIST_DIR}/VISPUse.cmake")

if(BUILD_TEST_COVERAGE)
  # Add build options for test coverage. Currently coverage is only supported
  # on gcc compiler
  # Because using -fprofile-arcs with shared lib can cause problems like:
  # hidden symbol `__bb_init_func', we add this option only for static
  # library build
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
endif()

#----------------------------------------------------------------------
# Remember VISP' third party libs configuration:
#----------------------------------------------------------------------
set(VISP_HAVE_AFMA4          "")
set(VISP_HAVE_AFMA6          "")
set(VISP_HAVE_APRILTAG       "TRUE")
set(VISP_HAVE_ARSDK          "")
set(VISP_HAVE_ATIDAQ         "")
set(VISP_HAVE_ATLAS          "")
set(VISP_HAVE_BICLOPS        "")
set(VISP_HAVE_CLIPPER        "TRUE")
set(VISP_HAVE_CMU1394        "")
set(VISP_HAVE_COIN3D         "")
set(VISP_HAVE_COIN3D_AND_GUI "")
set(VISP_HAVE_COMEDI         "")
set(VISP_HAVE_CPP11_COMPATIBILITY "") # to keep compat with previous releases
set(VISP_HAVE_CXX11          "")
set(VISP_CXX_STANDARD_98     "199711L")
set(VISP_CXX_STANDARD_11     "201103L")
set(VISP_CXX_STANDARD_14     "201402L")
set(VISP_CXX_STANDARD_17     "201703L")
set(VISP_CXX_STANDARD        "201103L")
set(VISP_HAVE_D3D9           "")
set(VISP_HAVE_DC1394         "")
set(VISP_HAVE_DIRECTSHOW     "")
set(VISP_HAVE_DISPLAY        "")
set(VISP_HAVE_DMTX           "")
set(VISP_HAVE_EIGEN3         "TRUE")
set(VISP_HAVE_FFMPEG         "")
set(VISP_HAVE_FRANKA         "")
set(VISP_HAVE_FLYCAPTURE     "")
set(VISP_HAVE_GDI            "TRUE")
set(VISP_HAVE_GTK            "")
set(VISP_HAVE_GSL            "")
set(VISP_HAVE_JPEG           "")
set(VISP_HAVE_LAPACK         "TRUE")
set(VISP_HAVE_LAPACK_C       "")
set(VISP_HAVE_LAPACK_BUILT_IN "TRUE")
set(VISP_HAVE_LAPACK_NETLIB  "")
set(VISP_HAVE_LIBFREENECT    "")
set(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES "")
set(VISP_HAVE_LIBFREENECT_OLD "")
set(VISP_HAVE_LIBUSB_1       "")
set(VISP_HAVE_MKL            "")
set(VISP_HAVE_OGRE           "")
set(VISP_HAVE_OIS            "")
set(VISP_HAVE_OPENBLAS       "")
set(VISP_HAVE_OPENCV         "TRUE")
set(VISP_HAVE_OPENCV_NONFREE "")
set(VISP_HAVE_OPENCV_VERSION "(4<<16 | 1<<8 | 1)")
set(VISP_HAVE_OPENGL         "1")
set(VISP_HAVE_OPENMP         "TRUE")
set(VISP_HAVE_PARPORT        "")
set(VISP_HAVE_PCL            "")
set(VISP_HAVE_PIONEER        "")
set(VISP_HAVE_PNG            "")
set(VISP_HAVE_PTHREAD        "")
set(VISP_HAVE_PTU46          "")
set(VISP_HAVE_PUGIXML        "TRUE")
set(VISP_HAVE_PYLON          "TRUE")
set(VISP_HAVE_QBDEVICE       "TRUE")
set(VISP_HAVE_QT             "")
set(VISP_HAVE_REALSENSE      "")
set(VISP_HAVE_REALSENSE2     "TRUE")
set(VISP_HAVE_SOQT           "")
set(VISP_HAVE_SOWIN          "")
set(VISP_HAVE_SOXT           "")
set(VISP_HAVE_V4L2           "")
set(VISP_HAVE_VIPER650       "")
set(VISP_HAVE_VIPER850       "")
set(VISP_HAVE_VIRTUOSE       "")
set(VISP_HAVE_X11            "")
set(VISP_HAVE_XML2           "")
set(VISP_HAVE_YARP           "")
set(VISP_HAVE_ZBAR           "")



set(VISP_COMPILER_FAIL_REGEX
    "command line option .* is valid for .* but not for C\\+\\+" # GNU
    "command line option .* is valid for .* but not for C" # GNU
    "unrecognized .*option"                     # GNU
    "unknown .*option"                          # Clang
    "ignoring unknown option"                   # MSVC
    "warning D9002"                             # MSVC, any lang
    "option .*not supported"                    # Intel
    "[Uu]nknown option"                         # HP
    "[Ww]arning: [Oo]ption"                     # SunPro
    "command option .* is not recognized"       # XL
    "not supported in this configuration; ignored"       # AIX
    "File with unknown suffix passed to linker" # PGI
    "WARNING: unknown flag:"                    # Open64
  )

#----------------------------------------------------------------------
# Propagate c++ standard compiler option if enabled during ViSP build
#----------------------------------------------------------------------
if(${VISP_CXX_STANDARD} EQUAL ${VISP_CXX_STANDARD_11})
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED FALSE)
  set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++11 instead of -std=gnu++11
  if(NOT CMAKE_CXX11_COMPILE_FEATURES)
    set(CXX11_CXX_FLAGS -std=c++11)
    if(CXX11_CXX_FLAGS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_CXX_FLAGS}")
    endif()
  endif()
elseif(${VISP_CXX_STANDARD} EQUAL ${VISP_CXX_STANDARD_14})
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED FALSE)
  set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++14 instead of -std=gnu++14
  if(NOT CMAKE_CXX14_COMPILE_FEATURES)
    set(CXX14_CXX_FLAGS -std=c++14)
    if(CXX14_CXX_FLAGS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX14_CXX_FLAGS}")
    endif()
  endif()
elseif(${VISP_CXX_STANDARD} EQUAL ${VISP_CXX_STANDARD_17})
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED FALSE)
  set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++17 instead of -std=gnu++17
  if(NOT CMAKE_CXX17_COMPILE_FEATURES)
    set(CXX14_CXX_FLAGS -std=c++14)
    if(CXX17_CXX_FLAGS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX17_CXX_FLAGS}")
    endif()
  endif()
endif()

#----------------------------------------------------------------------
# Propagate openmp compiler option if enabled during ViSP build
#----------------------------------------------------------------------
if(VISP_HAVE_OPENMP)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp")
endif()

#----------------------------------------------------------------------
# Propagate -fPIC option requested with Qt5 that requires position independent
# code if Qt was built with -reduce-relocations.
#----------------------------------------------------------------------
if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC") # Is needed for ANDROID too.
  endif()
endif()

#----------------------------------------------------------------------
# Some useful macro to be able to build the tutorials along side ViSP
#----------------------------------------------------------------------
# Create a target from the *.cpp file, link against ViSP libraries and add a dependency to ViSP library
# tu ensure that the library is build before this target.
macro(visp_add_target file_cpp)
  if(POLICY CMP0020)
    cmake_policy(PUSH)
    cmake_policy(SET CMP0020 NEW) # For UsTK: Qt5
  endif()
  get_filename_component(target ${file_cpp} NAME_WE)
  include_directories(${VISP_INCLUDE_DIRS})
  add_executable(${target} ${file_cpp} ${ARGN})
  target_link_libraries(${target} ${VISP_LIBRARIES})
  if(POLICY CMP0020)
    cmake_policy(POP) #cmake_policy(SET CMP0020 NEW) # For UsTK: Qt5
  endif()
endmacro()

# Create a dependency to the target extracted from the *.cpp file and put the target in the solution dependency folder.
macro(visp_add_dependency file_cpp dependency)
  get_filename_component(target ${file_cpp} NAME_WE)
  if(TARGET visp_${dependency})
    add_dependencies(visp_${dependency} ${target})
    if(ENABLE_SOLUTION_FOLDERS)
      set_target_properties(${target} PROPERTIES FOLDER "${dependency}")
    endif()
  endif()
endmacro()

# Copy the data files to the same location than the target associated to the cpp files
# Since CMake 3.0.0 policy CMP0026 was introduced to disallow location property on target.
# If CMake 3.0.0 is used, we use $<TARGET_FILE_DIR:tgt> to get the target location
if (CMAKE_VERSION VERSION_GREATER 2.8.12)
  macro(visp_copy_data file_cpp file_data)
    get_filename_component(target ${file_cpp} NAME_WE)
    get_filename_component(target_location "${target_location}" PATH)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${file_data}" "$<TARGET_FILE_DIR:${target}>"
    )
  endmacro()
else()
  macro(visp_copy_data file_cpp file_data)
    get_filename_component(target ${file_cpp} NAME_WE)
    get_target_property(target_location ${target} LOCATION)
    get_filename_component(target_location "${target_location}" PATH)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${file_data}" "${target_location}"
    )
  endmacro()
endif()

# Copy the data files to the same location than the target associated to the cpp files
# Since CMake 3.0.0 policy CMP0026 was introduced to disallow location property on target.
# If CMake 3.0.0 is used, we use $<TARGET_FILE_DIR:tgt> to get the target location
if (CMAKE_VERSION VERSION_GREATER 2.8.12)
  macro(visp_copy_dir file_cpp root_dir rel_dir)
    get_filename_component(target ${file_cpp} NAME_WE)
    get_filename_component(target_location "${target_location}" PATH)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${root_dir}/${rel_dir}" "$<TARGET_FILE_DIR:${target}>/${rel_dir}"
    )
  endmacro()
else()
  macro(visp_copy_data file_cpp root_dir rel_dir)
    get_filename_component(target ${file_cpp} NAME_WE)
    get_target_property(target_location ${target} LOCATION)
    get_filename_component(target_location "${target_location}" PATH)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${root_dir}/${rel_dir}" "${target_location}/${rel_dir}"
    )
  endmacro()
endif()

# Check dependencies for examples
macro(visp_check_dependencies)
  set(VP_DEPENDENCIES_FOUND TRUE)
  foreach(d ${ARGN})
    if(NOT TARGET ${d})
      #message(WARNING "ViSP: Can't resolve dependency: ${d}")
      set(VP_DEPENDENCIES_FOUND FALSE)
      break()
    endif()
  endforeach()
endmacro()

# add subdirectory if dependencies are found
# visp_add_subdirectory(<subdir> REQUIRED_DEPS <deps>)
macro(visp_add_subdirectory subdir)
  set(__deps "")

  foreach(arg ${ARGN})
    if(arg STREQUAL "REQUIRED_DEPS")
      set(__varname "__deps")
    else()
      list(APPEND ${__varname} ${arg})
    endif()
  endforeach()
  unset(__varname)

  if(__deps STREQUAL "")
    set(VP_DEPENDENCIES_FOUND TRUE)
  else()
    visp_check_dependencies(${__deps})
  endif()

  if(VP_DEPENDENCIES_FOUND)
    add_subdirectory(${subdir})
  endif()
endmacro()

# add specific compile flag useful to turn warning coming from 3rdparty
macro(visp_set_source_file_compile_flag file)
#  if(DEFINED ACTIVATE_WARNING_3PARTY_MUTE)
#    if(ACTIVATE_WARNING_3PARTY_MUTE)
  set(__cxxflags "")
  set(__lang "CXX")
  if("${file}" MATCHES "\\.(c)$")
    set(__lang "C")
  elseif("${file}" MATCHES "\\.(cpp|cc|cxx)$")
    set(__lang "CXX")
  elseif("${file}" MATCHES "\\.(f|F)$")
    set(__lang "Fortran")
  endif()
  # Since cxx11 option makes try_compile() result wrong, we remove all the CXX_FLAGS
  # when we check if an option is available or not
  #set(CXX_FLAGS_BACKUP ${CMAKE_CXX_FLAGS})
  #set(CMAKE_CXX_FLAGS "")
  foreach(cxxflag ${ARGN})
    visp_check_flag_support(${__lang} ${cxxflag} __support_flag "")
    if(${__support_flag})
      set(__cxxflags "${__cxxflags} ${cxxflag}")
    endif()
  endforeach()
  #set(CMAKE_CXX_FLAGS ${CXX_FLAGS_BACKUP})
  #unset(CXX_FLAGS_BACKUP)
  if(NOT ${__cxxflags} STREQUAL "")
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${file}")
      set_source_files_properties("${CMAKE_CURRENT_LIST_DIR}/${file}" PROPERTIES COMPILE_FLAGS "${__cxxflags}")
    elseif(EXISTS ${file}) # for files that are in the build tree (like those produced by qt moc)
      set_source_files_properties("${file}" PROPERTIES COMPILE_FLAGS "${__cxxflags}")
    endif()
  endif()
#    endif()
#  endif()
endmacro()

# check if a compiler flag is supported
macro(visp_check_flag_support lang flag varname base_options)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()

  if("_${lang}_" MATCHES "_CXX_")
    set(_lang CXX)
  elseif("_${lang}_" MATCHES "_C_")
    set(_lang C)
  elseif("_${lang}_" MATCHES "_Fortran_")
    set(_lang Fortran)
  else()
    set(_lang ${lang})
  endif()
  string(TOUPPER "${flag}" ${varname})
  string(REGEX REPLACE "^(/|-)" "HAVE_${_lang}_" ${varname} "${${varname}}")
  string(REGEX REPLACE " -|-|=| |\\." "_" ${varname} "${${varname}}")
  visp_check_compiler_flag("${_lang}" "${base_options} ${flag}" ${${varname}} ${ARGN})
endmacro()

# test if a compiler flag is supported
macro(visp_check_compiler_flag LANG FLAG RESULT)
  set(_fname "${ARGN}")
  if(NOT DEFINED ${RESULT})
    if(_fname)
      # nothing
    elseif("_${LANG}_" MATCHES "_CXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.cxx")
      #if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_C_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c")
      #if("${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main(void) { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main(void) { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_OBJCXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.mm")
      #if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_Fortran_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.f")
      file(WRITE "${_fname}" "      PROGRAM TEST\n      RETURN\n      END\n")
    else()
      unset(_fname)
    endif()
    if(_fname)
      if(NOT "x${ARGN}" STREQUAL "x")
        file(RELATIVE_PATH __msg "${CMAKE_SOURCE_DIR}" "${ARGN}")
        set(__msg " (check file: ${__msg})")
      else()
        set(__msg "")
      endif()
      message(STATUS "Performing Test ${RESULT}${__msg}")
      try_compile(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        CMAKE_FLAGS "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"   # CMP0056 do this on new CMake
        COMPILE_DEFINITIONS "${FLAG}"
        OUTPUT_VARIABLE OUTPUT)

      if(${RESULT})
        string(REPLACE ";" "," OUTPUT_LINES "${OUTPUT}")
        string(REPLACE "\n" ";" OUTPUT_LINES "${OUTPUT_LINES}")
        foreach(_regex ${VP_COMPILER_FAIL_REGEX})
          if(NOT ${RESULT})
            break()
          endif()
          foreach(_line ${OUTPUT_LINES})
            if("${_line}" MATCHES "${_regex}")
              file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
                  "Build output check failed:\n"
                  "    Regex: '${_regex}'\n"
                  "    Output line: '${_line}'\n")
              set(${RESULT} 0)
              break()
            endif()
          endforeach()
        endforeach()
      endif()

      if(${RESULT})
        set(${RESULT} 1 CACHE INTERNAL "Test ${RESULT}")
        message(STATUS "Performing Test ${RESULT} - Success")
      else()
        message(STATUS "Performing Test ${RESULT} - Failed")
        set(${RESULT} "" CACHE INTERNAL "Test ${RESULT}")
        file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Compilation failed:\n"
            "    source file: '${_fname}'\n"
            "    check option: '${FLAG}'\n"
            "===== BUILD LOG =====\n"
            "${OUTPUT}\n"
            "===== END =====\n\n")
      endif()
    else()
      set(${RESULT} 0)
    endif()
  endif()
endmacro()