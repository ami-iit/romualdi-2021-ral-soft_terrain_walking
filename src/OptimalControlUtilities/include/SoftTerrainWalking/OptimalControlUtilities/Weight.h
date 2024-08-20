/**
 * @file Weight.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H
#define SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H

#include <iDynTree/Core/VectorDynSize.h>

namespace SoftTerrainWalking
{
namespace OptimalControlUtilities
{
template <typename T> class Weight
{
    T m_rawWeight; /**< raw weight */
    double m_offset{0}; /**< Offset of the weight i.e. Weight = scaling * raw_weight + offset */
    double m_scaling{1}; /**< Scaling factor of the weight i.e. Weight = scaling * raw_weight +
                          offset*/

public:
    Weight(const T& rawWeight, const double& weightOffset, const double& weightScaling) noexcept;

    Weight(const T& rawWeight) noexcept;

    T getWeight() const noexcept;

    void setScaling(const double& scaling) noexcept;

    void setOffset(const double& offset) noexcept;

    void setRawWeight(const T& rawWeight) noexcept;

    constexpr static Weight<T> Zero(size_t size = 1) noexcept;
};
} // namespace OptimalControlUtilities
} // namespace SoftTerrainWalking

#include "Weight.tpp"

#endif // SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_WEIGHT_H
