# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name


add_soft_terrain_walking_library(
  NAME                   Simulator
  SOURCES                src/Simulator.cpp src/Visualizer.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/Simulator/Simulator.h include/SoftTerrainWalking/Simulator/Visualizer.h include/SoftTerrainWalking/Simulator/Plane.h
  PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler
                         BipedalLocomotion::ContinuousDynamicalSystem
                         SoftTerrainWalking::ContactModels
                         iDynTree::idyntree-core
                         iDynTree::idyntree-visualization)
