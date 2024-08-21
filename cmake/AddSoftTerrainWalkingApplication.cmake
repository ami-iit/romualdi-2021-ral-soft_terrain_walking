# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

function(add_soft_terrain_walking_application)

  set(options)
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    HEADERS
    LINK_LIBRARIES
    SUBDIRECTORIES)

  set(prefix "soft_terrain_walking_application")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(sources ${${prefix}_SOURCES})
  set(headers ${${prefix}_HEADERS})
  set(link_libraries ${${prefix}_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})

  # add an executable to the project using the specified source files.
  add_executable(${name} ${sources} ${headers})

  # Add C++17 features
  target_compile_features(${name} PRIVATE cxx_std_17)
  target_compile_definitions(${name} PRIVATE -D_USE_MATH_DEFINES)

  set_target_properties(${name} PROPERTIES
      OUTPUT_NAME "cmw-${name}"
      VERSION ${CentroidalMPCWalking_VERSION})

  target_link_libraries(${name} PRIVATE ${link_libraries})


  target_include_directories(${name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${name}
    DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

  # Add all subdirectories
  foreach(subdir ${subdirectories})
    add_subdirectory(${subdir})
  endforeach()

  message(STATUS "Created target application ${name}.")

endfunction()
