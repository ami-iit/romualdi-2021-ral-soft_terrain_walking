# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>



# set target name
set(EXE_TARGET_NAME SoftTerrainWalkingModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)


# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/Module.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/SoftTerrainWalking/Module/Module.h
  )

set(${EXE_TARGET_NAME}_THRIFT_HDR
  thrifts/SoftTerrainWalkingCommands.thrift
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Application target calls
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_GEN_FILES ${${EXE_TARGET_NAME}_THRIFT_HDR})

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR}
  ${${EXE_TARGET_NAME}_THRIFT_GEN_FILES})

target_link_libraries(${EXE_TARGET_NAME}
  WalkingControllers::StdUtilities
  WalkingControllers::TimeProfiler
  WalkingControllers::LoggerClient
  BipedalLocomotion::ParametersHandlerYarpImplementation
  BipedalLocomotion::Estimators
  SoftTerrainWalking::KinDynWrapper
  SoftTerrainWalking::TrajectoryPlanner
  SoftTerrainWalking::WholeBodyControllersRigid
  SoftTerrainWalking::WholeBodyControllers
  SoftTerrainWalking::Simulator
  SoftTerrainWalking::ContactModels
  )

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/app)
# add_subdirectory(app)
