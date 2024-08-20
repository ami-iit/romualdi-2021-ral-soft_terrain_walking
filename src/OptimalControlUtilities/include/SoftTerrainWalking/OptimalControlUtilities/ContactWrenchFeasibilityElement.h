/**
 * @file ContactWrenchFeasibilityElement.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTACT_WRENCH_FEASIBILITY_ELEMENT_H
#define SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTACT_WRENCH_FEASIBILITY_ELEMENT_H

#include <SoftTerrainWalking/OptimalControlUtilities/ControlProblemElements.h>
#include <SoftTerrainWalking/OptimalControlUtilities/VariableHandler.h>

namespace SoftTerrainWalking
{

namespace OptimalControlUtilities
{

/**
 */
class GeneralContactWrenchFeasibilityElement : public InequalityConstraintElement
{
protected:
    Frame<iDynTree::IndexRange, iDynTree::FrameIndex> m_frameInContact; /**< Frame in contact with the environment */
    iDynTree::Rotation m_rotationMatrix; /**< Frame rotation matrix */
    iDynTree::MatrixDynSize m_AInBodyFrame; /**< Constrain matrix written in body frame */
    double m_infinity; /**< Double representing the infinity */
    double m_minimalNormalForce; /**< Minimal normal force required */
    unsigned int m_nominalForceConstraintIndex; /**< Index of the minimal normal force required */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param frameInContact pair containing the frames in contact. The first
     * element is the name of the frame in the handler while the second is
     * the name in the model
     * @param numberOfPoints number of points used to approximate a quarter of the friction cone
     * @param staticFrictionCoefficient static friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient
     * @param minimalNormalForce minimal normal force required
     * @param footLimitX foot size on the X axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param footLimitY foot size on the Y axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param infinity double representing the infinity
     * @throw std::runtime_error if the frame is not defined
     */
    GeneralContactWrenchFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                           const VariableHandler& handler,
                                           const Frame<std::string, std::string>& frameInContact,
                                           const int& numberOfPoints,
                                           const double& staticFrictionCoefficient,
                                           const double& torsionalFrictionCoefficient,
                                           const double& minimalNormalForce,
                                           const iDynTree::Vector2& footLimitX,
                                           const iDynTree::Vector2& footLimitY,
                                           const double& infinity);
};

/**
 * ContactWrenchFeasibilityElement handles the constraints related to the contact wrenches
 * such that: Unilateral constraint, friction cone, torsional friction parameter
 * and position of the local Center of Pressure
 */
class ContactWrenchFeasibilityElement : public GeneralContactWrenchFeasibilityElement
{
public:

    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param frameInContact pair containing the frames in contact. The first
     * element is the name of the frame in the handler while the second is
     * the name in the model
     * @param numberOfPoints number of points used to approximate a quarter of the friction cone
     * @param staticFrictionCoefficient static friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient
     * @param minimalNormalForce minimal normal force required
     * @param footLimitX foot size on the X axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param footLimitY foot size on the Y axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param infinity double representing the infinity
     * @throw std::runtime_error if the frame is not defined
     */
    ContactWrenchFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                    const VariableHandler& handler,
                                    const Frame<std::string, std::string>& frameInContact,
                                    const int& numberOfPoints,
                                    const double& staticFrictionCoefficient,
                                    const double& torsionalFrictionCoefficient,
                                    const double& minimalNormalForce,
                                    const iDynTree::Vector2& footLimitX,
                                    const iDynTree::Vector2& footLimitY,
                                    const double& infinity);

    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;
};

/**
 * ContactWrenchFeasibilityElement handles the constraints related to the contact wrenches
 * such that: Unilateral constraint, friction cone, torsional friction parameter
 * and position of the local Center of Pressure
 */
class ContactWrenchRateOfChangeFeasibilityElement : public GeneralContactWrenchFeasibilityElement
{

    double m_samplingTime;
    iDynTree::VectorDynSize m_upperBoundForce;
    iDynTree::Wrench m_contatWrench;
    double m_lowerBoundNormalForce;

public:
    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);

    void setUpperBoundNormalForce(const double& upperForce);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    void setContactWrench(const iDynTree::Wrench& wrench);

    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param frameInContact pair containing the frames in contact. The first
     * element is the name of the frame in the handler while the second is
     * the name in the model
     * @param numberOfPoints number of points used to approximate a quarter of the friction cone
     * @param staticFrictionCoefficient static friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient
     * @param minimalNormalForce minimal normal force required
     * @param footLimitX foot size on the X axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param footLimitY foot size on the Y axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param infinity double representing the infinity
     * @throw std::runtime_error if the frame is not defined
     */
    ContactWrenchRateOfChangeFeasibilityElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                const VariableHandler& handler,
                                                const Frame<std::string, std::string>& frameInContact,
                                                const int& numberOfPoints,
                                                const double& staticFrictionCoefficient,
                                                const double& torsionalFrictionCoefficient,
                                                const double& minimalNormalForce,
                                                const iDynTree::Vector2& footLimitX,
                                                const iDynTree::Vector2& footLimitY,
                                                const double& infinity,
                                                const double& samplingTime);

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() final;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() final;

};


} // namespace OptimalControlUtilities
} // namespace SoftTerrainWalking

#endif // SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTACT_WRENCH_FEASIBILITY_ELEMENT_H
