/**
 * @file StableDCMModel.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef SOFT_TERRAIN_WALKING_TRAJECTORY_PLANNER_STABLE_DCM_MODEL_H
#define SOFT_TERRAIN_WALKING_TRAJECTORY_PLANNER_STABLE_DCM_MODEL_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>

namespace SoftTerrainWalking
{

namespace TrajectoryPlanner
{

/**
 * StableDCMModel linear inverted pendulum model.
 */
class StableDCMModel
{
    double m_omega; /**< Inverted time constant of the 3D-LIPM. */

    std::unique_ptr<iCub::ctrl::Integrator> m_comIntegrator{nullptr}; /**< CoM integrator object. */

    iDynTree::Vector2 m_dcmPosition; /**< Position of the DCM. */
    iDynTree::Vector2 m_zmpPosition; /**< Position of the ZMP. */
    iDynTree::Vector2 m_comPosition; /**< Position of the CoM. */
    iDynTree::Vector2 m_comVelocity; /**< Velocity of the CoM. */
    iDynTree::Vector2 m_comAcceleration; /**< Acceleration of the CoM. The acceleration is available
                                            only if the setZMPPosition is called*/

public:
    /**
     * Initialize the 3D-LIPM.
     * @param config config of the 3D-LIPM;
     * @return true on success, false otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the DCM).
     */
    void setDCMPosition(const iDynTree::Vector2& input);

    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the ZMP).
     */
    void setZMPPosition(const iDynTree::Vector2& input);

    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the DCM).
     */
    [[deprecated("Use setDCMPosition instead")]] void setInput(const iDynTree::Vector2& input);

    /**
     * Integrate the model.
     * @return true on success, false otherwise.
     */
    bool integrateModel();

    /**
     * Get the position of the CoM.
     * @return position of the CoM.
     */
    const iDynTree::Vector2& getCoMPosition() const;

    /**
     * Get the velocity of the CoM.
     * @return velocity of the CoM.
     */
    const iDynTree::Vector2& getCoMVelocity() const;

    /**
     * Get the acceleration of the CoM.
     * @return acceleration of the CoM.
     */
    const iDynTree::Vector2& getCoMAcceleration() const;

    /**
     * Reset the Model
     * @param initialValue initial position of the CoM
     * @return true/false in case of success/failure
     */
    bool reset(const iDynTree::Vector2& initialValue);
};
} // namespace TrajectoryPlanner
} // namespace SoftTerrainWalking

#endif
