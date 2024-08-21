/**
 * @file Simulator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef SOFT_TERRAIN_WALKING_SIMULATOR_SIMULATOR_H
#define SOFT_TERRAIN_WALKING_SIMULATOR_SIMULATOR_H

#include <memory>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>

#include <SoftTerrainWalking/ContactModels/ContinuousContactModel.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseDynamicsWithCompliantContacts.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

#include <random>

namespace SoftTerrainWalking
{
namespace Simulator
{
class Simulator
{
    struct Contact
    {
        std::shared_ptr<ContactModels::ContinuousContactModel> model;
        iDynTree::FrameIndex indexInTheModel;
        iDynTree::Transform frameNullForce;
        bool isInContact{true};
    };

    std::unordered_map<std::string, std::pair<const std::string, const iDynTree::Transform>>
        m_baseFrames;

    std::shared_ptr<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseDynamicsWithCompliantContacts>
        m_dynamicalSystem;
    std::unique_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseDynamicsWithCompliantContacts>>
        m_integrator;

    // TODO move from here
    double m_length, m_width, m_springCoeff, m_damperCoeff;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;

    int m_numberOfDoF;
    double m_robotTotalMass;
    double m_dT;
    decltype(std::chrono::system_clock::now()) m_t0{std::chrono::system_clock::now()};

    Contact m_leftContact;
    Contact m_rightContact;

    iDynTree::Vector3 m_gravity;

    iDynTree::VectorDynSize m_jointTorques;

    iDynTree::FrameIndex m_baseFrame;

    bool setBaseFrame(const iDynTree::FrameIndex& frameBaseIndex,
                      const std::string& name,
                      const iDynTree::Model& model);

    enum class State
    {
        NotInitialized,
        Initialized,
        Ready,
        Running
    };



    State m_state{State::NotInitialized};

public:

    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handlerWeak,
                    const iDynTree::Model& model);

    bool reset(const iDynTree::VectorDynSize& initialJointValues,
               const iDynTree::Transform& leftFootTransform,
               const iDynTree::Transform& rightFootTransform);

    bool advance(const double& time = 0);

    int getActuatedDoFs() const;

    iDynTree::Wrench leftWrench();
    iDynTree::Wrench rightWrench();

    const std::shared_ptr<const SoftTerrainWalking::ContactModels::ContinuousContactModel>
    leftContactModel() const;

    const std::shared_ptr<const SoftTerrainWalking::ContactModels::ContinuousContactModel>
    rightContactModel() const;

    const iDynTree::VectorDynSize jointPositions() const;
    const iDynTree::VectorDynSize jointVelocities() const;

    iDynTree::Transform baseTransform() const;
    iDynTree::Twist baseVelocity() const;

    void setLeftFootNullForceTransform(const iDynTree::Transform& transform);

    void setRightFootNullForceTransform(const iDynTree::Transform& transform);

    void setLeftFootState(bool isInContact);
    void setRightFootState(bool isInContact);

    bool setTorqueReferences(const iDynTree::VectorDynSize& torques);
    bool setAccelerationReferences(const iDynTree::VectorDynSize& acceleration);
    bool setVelocityReferences(const iDynTree::VectorDynSize& velocity);
};
} // namespace Simulator
} // namespace SoftTerrainWalking

#endif // SOFT_TERRAIN_WALKING_SIMULATOR_SIMULATOR_H
