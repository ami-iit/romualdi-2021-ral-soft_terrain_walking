# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>


add_soft_terrain_walking_library(
  NAME                   OptimalControlUtilitiesRigid
  SOURCES                 src/PID.cpp
                          src/VariableHandler.cpp
                          src/ControlProblemElements.cpp
                          src/OptimizationProblemElements.cpp
  PUBLIC_HEADERS          include/SoftTerrainWalking/OptimalControlUtilitiesRigid/PID.h
                          include/SoftTerrainWalking/OptimalControlUtilitiesRigid/VariableHandler.h
                          include/SoftTerrainWalking/OptimalControlUtilitiesRigid/ControlProblemElements.h
                          include/SoftTerrainWalking/OptimalControlUtilitiesRigid/OptimizationProblemElements.h
  PUBLIC_LINK_LIBRARIES  WalkingControllers::iDynTreeUtilities
                         Eigen3::Eigen)

