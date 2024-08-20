# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(H_PREFIX include/SoftTerrainWalking/OptimalControlUtilities)

add_soft_terrain_walking_library(
    NAME                   OptimalControlUtilities
    SOURCES                src/OptimizationProblemElements.cpp src/CentroidalMomentumElements.cpp src/PDController.cpp src/ControlProblemElements.cpp
                           src/RegularizationElements.cpp src/FeasibilityElements.cpp src/VariableHandler.cpp src/FloatingBaseMultiBodyDynamicsElements.cpp
                           src/ZMPElements.cpp src/CentroidalMomentumElementsWithCompliantContacts.cpp
                           src/CentroidalMomentumRateOfChangeElements.cpp
                           src/CentroidalMomentumRateOfChangeBounds.cpp
                           src/ContactModelElement.cpp src/ContactWrenchFeasibilityElement.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/CartesianElements.h  ${H_PREFIX}/CartesianElements.tpp
                           ${H_PREFIX}/PDController.h ${H_PREFIX}/PIDController.h
                           ${H_PREFIX}/PIDController.tpp ${H_PREFIX}/CentroidalMomentumElements.h
                           ${H_PREFIX}/PDController.tpp ${H_PREFIX}/ControlProblemElements.h
                           ${H_PREFIX}/RegularizationElements.h ${H_PREFIX}/FeasibilityElements.h
                           ${H_PREFIX}/VariableHandler.h
                           ${H_PREFIX}/FloatingBaseMultiBodyDynamicsElements.h
                           ${H_PREFIX}/ZMPElements.h ${H_PREFIX}/OptimizationProblemElements.h
                           ${H_PREFIX}/Frame.h ${H_PREFIX}/Frame.tpp ${H_PREFIX}/Weight.h
                           ${H_PREFIX}/Weight.tpp
                           ${H_PREFIX}/CentroidalMomentumElementsWithCompliantContacts.h
                           ${H_PREFIX}/CentroidalMomentumRateOfChangeElements.h
                           ${H_PREFIX}/CentroidalMomentumRateOfChangeBounds.h
                           ${H_PREFIX}/ContactModelElement.h
                           ${H_PREFIX}/ContactWrenchFeasibilityElement.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler
                           SoftTerrainWalking::ContactModels SoftTerrainWalking::Simulator
                           iDynTree::idyntree-core iDynTree::idyntree-core iDynTree::idyntree-high-level
    PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
