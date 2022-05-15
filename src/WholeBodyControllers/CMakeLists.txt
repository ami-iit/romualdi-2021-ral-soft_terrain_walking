# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/SoftTerrainWalking/WholeBodyControllers)

add_soft_terrain_walking_library(
  NAME                   WholeBodyControllers
  SOURCES                src/MomentumBasedControlHelper.cpp src/MomentumBasedControl.cpp
  PUBLIC_HEADERS         ${H_PREFIX}/MomentumBasedControlHelper.h ${H_PREFIX}/MomentumBasedControlHelper.tpp ${H_PREFIX}/MomentumBasedControl.h
  PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler  SoftTerrainWalking::OptimalControlUtilities OsqpEigen::OsqpEigen iDynTree::idyntree-core
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
