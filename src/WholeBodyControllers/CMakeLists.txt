# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(H_PREFIX include/SoftTerrainWalking/WholeBodyControllers)

add_soft_terrain_walking_library(
  NAME                   WholeBodyControllers
  SOURCES                src/MomentumBasedControlHelper.cpp src/MomentumBasedControl.cpp
  PUBLIC_HEADERS         ${H_PREFIX}/MomentumBasedControlHelper.h ${H_PREFIX}/MomentumBasedControlHelper.tpp ${H_PREFIX}/MomentumBasedControl.h
  PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler  SoftTerrainWalking::OptimalControlUtilities OsqpEigen::OsqpEigen iDynTree::idyntree-core
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
