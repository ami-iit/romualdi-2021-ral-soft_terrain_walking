# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
add_soft_terrain_walking_library(
  NAME                   ContactModels
  SOURCES                src/ContinuousContactModel.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/ContactModels/ContinuousContactModel.h
  PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core BipedalLocomotion::ParametersHandler BipedalLocomotion::ContactModels)
# SUBDIRECTORIES         tests)
