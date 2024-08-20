/**
 * @file Plane.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef SOFT_TERRAIN_WALKING_SIMULATOR_PLANE_H
#define SOFT_TERRAIN_WALKING_SIMULATOR_PLANE_H

#include <string>

namespace SoftTerrainWalking
{
namespace Simulator
{
std::string getPlaneModel()
{
    std::string model = R"zzz(<?xml version="1.0" ?>
                         <robot name="plane">
                             <link name="world"/>
                             <link name="s_0">
                                 <inertial>
                                    <origin xyz="5.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
                                    <mass value="1"/>
                                      <inertia ixx="33.33354166666666" ixy="0.0" ixz="0.0" iyy="33.33354166666666" iyz="0.0" izz="66.66666666666666"/>);
                                 </inertial>"
                                 <visual>
                                    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                                     <geometry>
                                         <box size="20.0 20.0 0.10"/>
                                     </geometry>
                                 </visual>
                                 <collision>
                                    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                                    <geometry>
                                        <box size="20.0 20.0 0.10"/>
                                    </geometry>
                                </collision>
                             </link>
                             <joint name="joint_world_s_0" type="fixed">
                                 <origin xyz="0 0 0.05" rpy="0 0 0"/>
                                 <parent link="world"/>
                                 <child link="s_0"/>
                             </joint>
                         </robot>)zzz";

    return model;
};
} // namespace Simulator
} // namespace SoftTerrainWalking
#endif
