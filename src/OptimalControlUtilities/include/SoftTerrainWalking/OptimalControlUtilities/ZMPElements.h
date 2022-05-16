/**
 * @file ZMPElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H
#define SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H

#include <SoftTerrainWalking/OptimalControlUtilities/ControlProblemElements.h>
#include <SoftTerrainWalking/OptimalControlUtilities/VariableHandler.h>

namespace SoftTerrainWalking
{

namespace OptimalControlUtilities
{

/**
 * ZMPElement handles the tracking of the global Zero Moment Point (ZMP)
 */
class ZMPElement : public ControlTask
{
    /** Vectors containing the frames in contact with the environment */
    std::vector<Frame<iDynTree::IndexRange, iDynTree::FrameIndex>> m_framesInContact;
    iDynTree::Vector2 m_ZMP; /**< Desired global Zero Moment Point position */
    iDynTree::Position m_contactFramePosition; /**< Position of the contact frame */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact.
     * @throw std::runtime_error if the frame is not defined
     */
    ZMPElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
               const VariableHandler& handler,
               const std::vector<Frame<std::string, std::string>>& framesInContact);

    /**
     * Set the desired ZMP
     * @param ZMP position of the zero moment point
     */
    void setDesiredZMP(const iDynTree::Vector2& ZMP);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;
};

} // namespace OptimalControlUtilities
} // namespace SoftTerrainWalking

#endif // SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_ZMP_ELEMENT_H
