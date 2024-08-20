/**
 * @file CentroidalMomentumRateOfChangeElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <SoftTerrainWalking/OptimalControlUtilities/CentroidalMomentumRateOfChangeElements.h>

#include <iDynTree/Model/Model.h>

#include <limits>
#include <numeric>

using namespace SoftTerrainWalking::OptimalControlUtilities;

CentroidalLinearMomentumRateOfChangeElement::CentroidalLinearMomentumRateOfChangeElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    PIDController<iDynTree::Vector3> controller,
    const VariableHandler& handler,
    const FramesInContact& framesInContact)
    : ControlTask(kinDyn)
    , m_pid(controller)
{
    m_name = "Centroidal Linear Momentum rate of change Element [ ";

    // get the robot weight expressed in the inertial frame. (The z axis points upwards)
    m_robotMass = m_kinDynPtr->model().getTotalMass();
    double gravity = 9.81;
    m_robotWeight.zero();
    m_robotWeight(2) = -gravity * m_robotMass;

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    for (const auto& frame : framesInContact)
    {
        const auto& frameNameInVariableHandler = frame.identifierInVariableHandler();
        const auto& frameNameInModel = frame.identifierInModel();
        const auto& indexInVariableHandler = handler.getVariable(frameNameInVariableHandler);
        const auto& indexInModel = m_kinDynPtr->model().getFrameIndex(frameNameInModel);

        m_name += "( Label: " + frameNameInVariableHandler + " ) ";

        if (!indexInVariableHandler.isValid())
            throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                     "CentroidalLinearMomentumElementWithCompliantContact] "
                                     "Undefined label "
                                     + frameNameInVariableHandler + " in the variablehandler");

        // set constant elements in the A matrix
        // A = [ I O I O ...]
        iDynTree::toEigen(m_A).middleCols<3>(indexInVariableHandler.offset).setIdentity();

        m_framesInContact.insert(
            {frameNameInVariableHandler,
             {indexInVariableHandler, indexInModel, /*compliant contact = */ true}});
    }

    m_name += "]";
}

const iDynTree::VectorDynSize& CentroidalLinearMomentumRateOfChangeElement::getB()
{
    // Get the output of the PD controller
    m_b = m_pid.getControllerOutput();
    return m_b;
}

void CentroidalLinearMomentumRateOfChangeElement::setReference(
    const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
    const iDynTree::Vector3& centroidalLinearMomentumDerivative,
    const iDynTree::Vector3& centroidalLinearMomentum,
    const iDynTree::Vector3& centerOfMass) noexcept
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    m_pid.setReference(centroidalLinearMomentumSecondDerivative,
                       centroidalLinearMomentumDerivative,
                       centroidalLinearMomentum,
                       centerOfMass);
}

bool CentroidalLinearMomentumRateOfChangeElement::setMeasuredContactWrenches(
    const std::unordered_map<std::string ,iDynTree::Wrench>& contactWrenches)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)
    //    + ki (com_des - com)

    // The centroidalLinearMomentumDerivative is equal to the sum of the contact forces and the
    // weight acting on the system
    auto centroidalLinearMomentumDerivative = m_robotWeight;

    // check for all frame in contact the corresponding contact wrench. If the frame is not
    // associated to any contact wrenches, the function will return an error
    for(auto& frameInContact : m_framesInContact)
    {
        const std::string& key = frameInContact.first;
        auto& frame = frameInContact.second;

        // check if the key is associated to a frame
        const auto& contactWrench = contactWrenches.find(key);
        if (contactWrench == contactWrenches.end())
        {
            std::cerr << "[CentroidalLinearMomentumRateOfChangeElement::"
                         "setMeasuredContactWrenches] The label "
                      << key << " is not associated to any contact wrench" << std::endl;
            return false;
        }
        const iDynTree::Wrench& wrench = contactWrench->second;

        iDynTree::toEigen(centroidalLinearMomentumDerivative)
            += iDynTree::toEigen(wrench.getLinearVec3());
    }

    m_pid.setFeedback(centroidalLinearMomentumDerivative,
                      m_kinDynPtr->getCentroidalTotalMomentum().getLinearVec3(),
                      m_kinDynPtr->getCenterOfMassPosition());

    return true;
}

void CentroidalLinearMomentumRateOfChangeElement::setGains(const iDynTree::Vector3& kd,
                                                           const iDynTree::Vector3& kp,
                                                           const iDynTree::Vector3& ki)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    m_pid.setGains(kd, kp, ki);
}


CentroidalAngularMomentumRateOfChangeElement::CentroidalAngularMomentumRateOfChangeElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    PIDController<iDynTree::Vector3> controller,
    const VariableHandler& handler,
    const FramesInContact& framesInContact,
    const double& dT)
    : ControlTask(kinDyn)
    , m_pid(controller)
{
    // required for the integrator
    m_name = "Centroidal Angular Momentum rate of change Element [ ";

    m_dT = dT;
    m_angularMomentumIntegral.zero();
    m_desiredAngularMomentumIntegral.zero();

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    for (const auto& frame : framesInContact)
    {
        const auto& frameNameInVariableHandler = frame.identifierInVariableHandler();
        const auto& frameNameInModel = frame.identifierInModel();
        const auto& indexInVariableHandler = handler.getVariable(frameNameInVariableHandler);
        const auto& indexInModel = m_kinDynPtr->model().getFrameIndex(frameNameInModel);

        m_name += "( Label: " + frameNameInVariableHandler + ", Frame: " + frameNameInModel + " ) ";

        if (!indexInVariableHandler.isValid())
            throw std::runtime_error("[CentroidalAngularMomentumElementWithCompliantContact::"
                                     "CentroidalAngularMomentumElementWithCompliantContact] "
                                     "Undefined label "
                                     + frameNameInVariableHandler + " in the variablehandler");

        if (indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CentroidalAngularMomentumElementWithCompliantContact::"
                                     "CentroidalLineartMomentumElementWithCompliantContact] "
                                     "Undefined farme "
                                     + frameNameInModel + " in the model");

        // set constant elements in the A matrix
        // A = [ * I * I ...]
        iDynTree::toEigen(m_A).middleCols<3>(indexInVariableHandler.offset + 3).setIdentity();

        m_framesInContact.insert(
            {frameNameInVariableHandler,
             {indexInVariableHandler, indexInModel, /*compliant contact = */ true}});
    }

    m_name += "]";
}

const iDynTree::VectorDynSize& CentroidalAngularMomentumRateOfChangeElement::getB()
{
    using iDynTree::toEigen;

    // Get the output of the PD controller
    m_b = m_pid.getControllerOutput();

    iDynTree::Vector3 comVelocity = m_kinDynPtr->getCenterOfMassVelocity();
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;
        toEigen(m_b)
            -= iDynTree::skew(
                   toEigen(m_kinDynPtr->getFrameVel(frame.identifierInModel()).getLinearVec3())
                   - toEigen(comVelocity))
               * toEigen(frame.contactWrench().getLinearVec3());
    }

    return m_b;
}

const iDynTree::MatrixDynSize& CentroidalAngularMomentumRateOfChangeElement::getA()
{
    using iDynTree::toEigen;

    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        toEigen(m_A).middleCols<3>(frame.identifierInVariableHandler().offset) = iDynTree::skew(
            toEigen(m_kinDynPtr->getWorldTransform(frame.identifierInModel()).getPosition())
            - toEigen(com));
    }

    return m_A;
}

void CentroidalAngularMomentumRateOfChangeElement::setReference(
    const iDynTree::Vector3& centroidalAngularMomentumSecondDerivative,
    const iDynTree::Vector3& centroidalAngularMomentumDerivative,
    const iDynTree::Vector3& centroidalAngularMomentum)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    iDynTree::toEigen(m_desiredAngularMomentumIntegral) += iDynTree::toEigen(centroidalAngularMomentum) * m_dT;
    m_pid.setReference(centroidalAngularMomentumSecondDerivative,
                       centroidalAngularMomentumDerivative,
                       centroidalAngularMomentum,
                       m_desiredAngularMomentumIntegral);
}

bool CentroidalAngularMomentumRateOfChangeElement::setMeasuredContactWrenches(
    const std::unordered_map<std::string ,iDynTree::Wrench>& contactWrenches)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    iDynTree::Vector3 angularMomentumDerivative;
    angularMomentumDerivative.zero();

    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    // check for all frame in contact the corresponding contact wrench
    for(auto& frameInContact : m_framesInContact)
    {
        const std::string& key = frameInContact.first;
        auto& frame = frameInContact.second;

        // check if the key is associated to a frame
        const auto& contactWrench = contactWrenches.find(key);
        if (contactWrench == contactWrenches.end())
        {
            std::cerr << "[CentroidalAngularMomentumRateOfChangeElement::"
                         "setMeasuredContactWrenches] The label "
                      << key << " is not associated to any contact wrench" << std::endl;
            return false;
        }
        const iDynTree::Wrench& wrench = contactWrench->second;

        // store wrench
        frame.contactWrench() = wrench;

        // compute angular momentum derivative
        iDynTree::toEigen(angularMomentumDerivative)
            += iDynTree::toEigen(wrench.getAngularVec3())
               + iDynTree::skew(toEigen(
                     m_kinDynPtr->getWorldTransform(frame.identifierInModel()).getPosition() - com))
                     * iDynTree::toEigen(wrench.getLinearVec3());
    }

    iDynTree::toEigen(m_angularMomentumIntegral)
        += iDynTree::toEigen(m_kinDynPtr->getCentroidalTotalMomentum().getAngularVec3()) * m_dT;

    m_pid.setFeedback(angularMomentumDerivative,
                      m_kinDynPtr->getCentroidalTotalMomentum().getAngularVec3(),
                      m_angularMomentumIntegral);

    return true;
}

void CentroidalAngularMomentumRateOfChangeElement::setGains(const iDynTree::Vector3& kd,
                                                            const iDynTree::Vector3& kp,
                                                            const iDynTree::Vector3& ki)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    m_pid.setGains(kd, kp, ki);
}
