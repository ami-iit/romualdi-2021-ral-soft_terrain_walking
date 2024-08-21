# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# set target name
add_soft_terrain_walking_library(
  NAME                   ContactModels
  SOURCES                src/ContinuousContactModel.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/ContactModels/ContinuousContactModel.h
  PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core BipedalLocomotion::ParametersHandler BipedalLocomotion::ContactModels)
# SUBDIRECTORIES         tests)
