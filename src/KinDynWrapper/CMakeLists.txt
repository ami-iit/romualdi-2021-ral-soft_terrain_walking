# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_soft_terrain_walking_library(
  NAME                   KinDynWrapper
  SOURCES                src/Wrapper.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/KinDynWrapper/Wrapper.h
  PUBLIC_LINK_LIBRARIES  WalkingControllers::YarpUtilities
                         ${iDynTree_LIBRARIES}
                         ctrlLib
                         Eigen3::Eigen)
