# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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
