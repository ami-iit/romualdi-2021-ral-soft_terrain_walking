# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>


add_soft_terrain_walking_library(
  NAME                   WholeBodyControllersRigid
  SOURCES                src/InverseKinematics.cpp
                         src/QPInverseKinematics.cpp
                         src/QPInverseKinematics_osqp.cpp
                         src/QPInverseKinematics_qpOASES.cpp
                         src/TaskBasedTorqueControl.cpp
  PUBLIC_HEADERS         include/SoftTerrainWalking/WholeBodyControllersRigid/InverseKinematics.h
                         include/SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics.h
                         include/SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics_osqp.h
                         include/SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics_qpOASES.h
                         include/SoftTerrainWalking/WholeBodyControllersRigid/TaskBasedTorqueControl.h
  PUBLIC_LINK_LIBRARIES  WalkingControllers::YarpUtilities
                         WalkingControllers::iDynTreeUtilities
                         SoftTerrainWalking::OptimalControlUtilitiesRigid
                         osqp::osqp
                         OsqpEigen::OsqpEigen
                         ${qpOASES_LIBRARIES}
                         ctrlLib)
