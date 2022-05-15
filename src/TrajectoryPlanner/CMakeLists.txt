# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_soft_terrain_walking_library(
  NAME                   TrajectoryPlanner
  SOURCES                src/StableDCMModel.cpp src/TrajectoryGenerator.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/TrajectoryPlanner/StableDCMModel.h
                         include/SoftTerrainWalking/TrajectoryPlanner/TrajectoryGenerator.h
  PUBLIC_LINK_LIBRARIES  Threads::Threads
                         WalkingControllers::YarpUtilities
                         UnicyclePlanner
                         ctrlLib
                         Eigen3::Eigen)
