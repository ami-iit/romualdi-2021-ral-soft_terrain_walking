/**
 * @file Simulator.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/CompliantContactWrench.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseDynamicsWithCompliantContacts.h>
#include <SoftTerrainWalking/Simulator/Simulator.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixView.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>

using namespace SoftTerrainWalking::Simulator;

int Simulator::getActuatedDoFs() const
{
    return m_numberOfDoF;
}

bool Simulator::initialize(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handlerWeak,
    const iDynTree::Model& model)
{

    m_numberOfDoF = model.getNrOfDOFs();
    m_robotTotalMass = model.getTotalMass();


    auto handler = handlerWeak.lock();
    if(handler == nullptr)
    {
        std::cerr << "[Simulator::initialize] The parameter handler is not valid."
                  << std::endl;
        return false;
    }

    if(m_state != State::NotInitialized)
    {
        std::cerr << "[Simulator::initialize] The simulator has been already initialized."
                  << std::endl;
        return false;
    }

    // get contact models parameters
    // TODO please remove me
    if (!handler->getParameter("length", m_length) || !handler->getParameter("width", m_width)
        || !handler->getParameter("spring_coeff", m_springCoeff)
        || !handler->getParameter("damper_coeff", m_damperCoeff))
    {
        std::cerr << "[Simulator::initialize] Unable to get the contact parameters." << std::endl;
        return false;
    }

    // Initialize the left foot contact model
    m_leftContact.model = std::make_shared<ContactModels::ContinuousContactModel>(true);
    if(!m_leftContact.model->initialize(handlerWeak))
    {
        std::cerr << "[Simulator::initialize] Unable to initialize the left foot contact model." << std::endl;
        return false;
    }

    m_rightContact.model = std::make_unique<ContactModels::ContinuousContactModel>(true);
    if(!m_rightContact.model->initialize(handlerWeak))
    {
        std::cerr << "[Simulator::initialize] Unable to initialize the right foot contact model." << std::endl;
        return false;
    }

    // get the contact frames
    std::string footFrame;
    if (!handler->getParameter("left_foot_frame", footFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_leftContact.indexInTheModel = model.getFrameIndex(footFrame);
    if (m_leftContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[Simulator::initialize] Unable to find the frame named: " << footFrame
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("right_foot_frame", footFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_rightContact.indexInTheModel = model.getFrameIndex(footFrame);
    if (m_rightContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[Simulator::initialize] Unable to find the frame named: " << footFrame
                  << std::endl;
        return false;
    }

    if (!setBaseFrame(m_leftContact.indexInTheModel, "left_foot", model))
    {
        std::cerr << "[Simulator::initialize] Unable to set the leftFootFrame." << std::endl;
        return false;
    }

    // set the right foot frame
    if (!setBaseFrame(m_rightContact.indexInTheModel, "right_foot", model))
    {
        std::cerr << "[Simulator::initialize] Unable to set the rightFootFrame." << std::endl;
        return false;
    }

    // get the base frame
    std::string baseFrame;
    if (!handler->getParameter("base_frame", baseFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_baseFrame = model.getFrameIndex(baseFrame);

    if (!setBaseFrame(m_baseFrame, "root", model))
    {
        std::cerr << "[Simulator::initialize] Unable to set the root frame." << std::endl;
        return false;
    }

    if (!handler->getParameter("sampling_time", m_dT))
    {
        std::cerr << "[Simulator::initialize] Unable to get the sampling time." << std::endl;
        return false;
    }


    // instantiate the integrator
    m_integrator = std::make_unique<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseDynamicsWithCompliantContacts>>();
    m_integrator->setIntegrationStep(m_dT);

    // instantiate the dynamical system
    m_dynamicalSystem = std::make_shared<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseDynamicsWithCompliantContacts>();

    if (!m_dynamicalSystem->setRobotModel(model))
    {
        std::cerr << "[Simulator::initialize] Unable to set the robot model in the dynamical "
                     "system."
                  << std::endl;
        return false;
    }

    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    m_kinDyn->loadRobotModel(model);
    m_kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    m_state = State::Initialized;

    return true;
}


bool Simulator::reset(const iDynTree::VectorDynSize& initialJointValues,
                      const iDynTree::Transform& leftFootTransform,
                      const iDynTree::Transform& rightFootTransform)
{
    if (m_state != State::Initialized && m_state != State::Ready && m_state != State::Running)
    {
        std::cerr << "[Simulator::reset] The simulator cannot be reset. Please initialize it"
                  << std::endl;
        return false;
    }

    iDynTree::VectorDynSize jointVelocity(initialJointValues.size());
    jointVelocity.zero();


    // compute the base position
    // -mg + length * width * k (-z) = 0
    // z = -mg / (length * width * k);
    m_gravity(0) = 0;
    m_gravity(1) = 0;
    m_gravity(2) = -9.81;

    iDynTree::Position leftFootPosition = leftFootTransform.getPosition();
    leftFootPosition(2) = -m_robotTotalMass * (-m_gravity(2)) / (2 * m_length * m_width * m_springCoeff);

    iDynTree::Transform leftFootTransformOnCarpet(leftFootTransform.getRotation(),
                                                  leftFootPosition);

    if (!m_kinDyn->setFloatingBase(m_baseFrames["left_foot"].first))
    {
        std::cerr << "[Simulator::reset] Unable to set the floating base" << std::endl;
        return false;
    }

    iDynTree::Twist baseTwist;
    baseTwist.zero();
    if (!m_kinDyn->setRobotState(leftFootTransformOnCarpet * m_baseFrames["left_foot"].second,
                                 initialJointValues,
                                 baseTwist,
                                 jointVelocity,
                                 m_gravity))
    {
        std::cerr << "[Simulator::reset] Unable to set the kinDynObject" << std::endl;
        return false;
    }

    // get the base transform
    const auto baseTransform = m_kinDyn->getWorldTransform(m_baseFrame);
    const auto& basePosition = baseTransform.getPosition();
    const auto& baseRotation = baseTransform.getRotation();

    if (!m_kinDyn->setFloatingBase(m_baseFrames["root"].first))
    {
        std::cerr << "[Simulator::reset] Unable to set the floating base" << std::endl;
        return false;
    }

    if (!m_kinDyn->setRobotState(baseTransform * m_baseFrames["root"].second,
                                 initialJointValues,
                                 baseTwist,
                                 jointVelocity,
                                 m_gravity))
    {
        std::cerr << "[Simulator::reset] Unable to set the kinDynObject" << std::endl;
        return false;
    }

    // contact model
    m_leftContact.frameNullForce = leftFootTransform;
    m_rightContact.frameNullForce = rightFootTransform;

    // compute contact wrench
    m_leftContact.model->setState(m_kinDyn->getFrameVel(m_leftContact.indexInTheModel),
                                  m_kinDyn->getWorldTransform(m_leftContact.indexInTheModel));

    // todo probably can be moved from here
    m_leftContact.model->setNullForceTransform(m_leftContact.frameNullForce);

    // test position Waring spring
    const double left_springCoeff
        = m_springCoeff - 100000 * m_leftContact.frameNullForce.getPosition()(0);
    m_leftContact.model->springCoeff() = left_springCoeff;

    m_rightContact.model->setState(m_kinDyn->getFrameVel(m_rightContact.indexInTheModel),
                                   m_kinDyn->getWorldTransform(m_rightContact.indexInTheModel));

    // todo probably can be moved from here
    m_rightContact.model->setNullForceTransform(m_rightContact.frameNullForce);

    // test position Waring spring
    const double right_springCoeff
        = m_springCoeff - 100000 * m_rightContact.frameNullForce.getPosition()(0);
    m_rightContact.model->springCoeff() = right_springCoeff;


    // set the Dynamical system
    m_dynamicalSystem->setState({iDynTree::toEigen(baseTwist.asVector()),
                                 iDynTree::toEigen(jointVelocity),
                                 iDynTree::toEigen(basePosition),
                                 iDynTree::toEigen(baseRotation),
                                 iDynTree::toEigen(initialJointValues)});

    // set the integrator
    m_integrator->setDynamicalSystem(m_dynamicalSystem);


    m_state = State::Ready;

    return true;
}

bool Simulator::advance(const double& time)
{
    using iDynTree::toEigen;

    if (m_state != State::Ready && m_state != State::Running)
    {
        std::cerr << "[Simulator::advance] The simulator cannot advance. Please reset it."
                  << std::endl;
        return false;
    }

    // the contact wrenches has been computed at the previous step
    std::vector<BipedalLocomotion::ContinuousDynamicalSystem::CompliantContactWrench>
        contactWrenches;
    if(m_leftContact.isInContact)
        contactWrenches.push_back(
            BipedalLocomotion::ContinuousDynamicalSystem::
                CompliantContactWrench(m_leftContact.indexInTheModel, m_leftContact.model));
    if (m_rightContact.isInContact)
        contactWrenches.push_back(
            BipedalLocomotion::ContinuousDynamicalSystem::
                CompliantContactWrench(m_rightContact.indexInTheModel, m_rightContact.model));

    m_dynamicalSystem->setControlInput({iDynTree::toEigen(m_jointTorques), contactWrenches});

    if (!m_integrator->integrate(0, time))
    {
        std::cerr << "[Simulator::advance] Unable to integrate the dynamics" << std::endl;
        return false;
    }

    const auto & [baseTwist, jointVelocity, basePosition, baseRotation, jointPosition] = m_integrator->getSolution();

    // update kindyn
    iDynTree::Twist twist;
    iDynTree::toEigen(twist.getLinearVec3()) = baseTwist.head<3>();
    iDynTree::toEigen(twist.getAngularVec3()) = baseTwist.tail<3>();

    iDynTree::Transform baseTransform;
    baseTransform.setPosition(iDynTree::make_span(basePosition));
    baseTransform.setRotation(iDynTree::make_matrix_view(baseRotation));

    if (!m_kinDyn->setRobotState(baseTransform,
                                 iDynTree::make_span(jointPosition),
                                 twist,
                                 iDynTree::make_span(jointVelocity),
                                 m_gravity))
    {
        std::cerr << "[Simulator::advance] Unable to set the kinDynObject at iteration number "
                  << std::endl;
        return false;
    }

    // auto now = std::chrono::system_clock::now();
    // double diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_t0).count() / 1000.0;

    // std::cerr << diff << std::endl;

    // const double right_springCoeff = m_springCoeff; // + 100000 * sin(2 * M_PI /100 * diff);
    // const double left_springCoeff = m_springCoeff; // + 100000 * sin(2 * M_PI /100 *  diff);

    // test position Waring springg
    const double right_damperCoeff
        = m_damperCoeff - (m_damperCoeff - 2e3) * m_rightContact.frameNullForce.getPosition()(0);
    const double right_springCoeff
        = m_springCoeff - (m_springCoeff - 5e5) * m_rightContact.frameNullForce.getPosition()(0);

    // std::cerr << "coeff right " << right_damperCoeff << std::endl;
    m_rightContact.model->springCoeff() = right_springCoeff;
    m_rightContact.model->damperCoeff() = right_damperCoeff;

    const double left_damperCoeff
        = m_damperCoeff - (m_damperCoeff - 2e3) * m_leftContact.frameNullForce.getPosition()(0);
    const double left_springCoeff
        = m_springCoeff - (m_springCoeff - 5e5) * m_leftContact.frameNullForce.getPosition()(0);
    // std::cerr << "coeff left " << left_damperCoeff << std::endl;
    m_leftContact.model->springCoeff() = left_springCoeff;
    m_leftContact.model->damperCoeff() = left_damperCoeff;

    m_leftContact.model->setState(m_kinDyn->getFrameVel(m_leftContact.indexInTheModel),
                                  m_kinDyn->getWorldTransform(m_leftContact.indexInTheModel));

    // todo probably can be moved from here
    m_leftContact.model->setNullForceTransform(m_leftContact.frameNullForce);


    m_rightContact.model->setState(m_kinDyn->getFrameVel(m_rightContact.indexInTheModel),
                                   m_kinDyn->getWorldTransform(m_rightContact.indexInTheModel));

    // todo probably can be moved from here
    m_rightContact.model->setNullForceTransform(m_rightContact.frameNullForce);

    return true;
}

bool Simulator::setTorqueReferences(const iDynTree::VectorDynSize& torques)
{
    if (torques.size() != m_numberOfDoF)
    {
        std::cerr << "[Simulator::setTorqueReferences] The number of desired torques is different "
                     "from the number of DoFs. Number of DoF "
                  << m_numberOfDoF << " size of desired torque vector " << torques.size()
                  << std::endl;
        return false;
    }

    m_jointTorques = torques;

    return true;
}

void Simulator::setLeftFootNullForceTransform(const iDynTree::Transform& transform)
{
    m_leftContact.frameNullForce = transform;

    // compute contact wrench
    m_leftContact.model->setState(m_kinDyn->getFrameVel(m_leftContact.indexInTheModel),
                                  m_kinDyn->getWorldTransform(m_leftContact.indexInTheModel));

    m_leftContact.model->setNullForceTransform(m_leftContact.frameNullForce);
}

void Simulator::setRightFootNullForceTransform(const iDynTree::Transform& transform)
{
    m_rightContact.frameNullForce = transform;

    // todo probably we can remove
    m_rightContact.model->setState(m_kinDyn->getFrameVel(m_rightContact.indexInTheModel),
                                   m_kinDyn->getWorldTransform(m_rightContact.indexInTheModel));

    m_rightContact.model->setNullForceTransform(m_rightContact.frameNullForce);
}

const std::shared_ptr<const SoftTerrainWalking::ContactModels::ContinuousContactModel>
Simulator::leftContactModel() const
{
    return m_leftContact.model;
}

const std::shared_ptr<const SoftTerrainWalking::ContactModels::ContinuousContactModel>
Simulator::rightContactModel() const
{
    return m_rightContact.model;
}

iDynTree::Wrench Simulator::leftWrench()
{
    if(!m_leftContact.isInContact)
        return iDynTree::Wrench::Zero();

    iDynTree::Wrench wrench = m_leftContact.model->getContactWrench();

    return wrench;
}

iDynTree::Wrench Simulator::rightWrench()
{
    if(!m_rightContact.isInContact)
        return iDynTree::Wrench::Zero();

    iDynTree::Wrench wrench = m_rightContact.model->getContactWrench();

// -
    return wrench;
}

void Simulator::setLeftFootState(bool isInContact)
{
    m_leftContact.isInContact = isInContact;
}

void Simulator::setRightFootState(bool isInContact)
{
    m_rightContact.isInContact = isInContact;
}

const iDynTree::VectorDynSize Simulator::jointPositions() const
{
    iDynTree::VectorDynSize v(m_numberOfDoF);
    iDynTree::toEigen(v) = std::get<4>(m_integrator->getSolution());
    return v;
}

const iDynTree::VectorDynSize Simulator::jointVelocities() const
{
    iDynTree::VectorDynSize v(m_numberOfDoF);
    iDynTree::toEigen(v) = std::get<1>(m_integrator->getSolution());
    return v;
}

iDynTree::Transform Simulator::baseTransform() const
{
    const iDynTree::Rotation baseRotation
        = iDynTree::make_matrix_view(std::get<3>(m_integrator->getSolution()));
    const iDynTree::Position& basePosition
        = iDynTree::make_span(std::get<2>(m_integrator->getSolution()));
    return iDynTree::Transform(baseRotation, basePosition);
}

iDynTree::Twist Simulator::baseVelocity() const
{
    iDynTree::Twist twist;
    iDynTree::toEigen(twist.getLinearVec3()) = std::get<0>(m_integrator->getSolution()).head<3>();
    iDynTree::toEigen(twist.getAngularVec3()) = std::get<0>(m_integrator->getSolution()).tail<3>();
    return twist;
}

bool Simulator::setBaseFrame(const iDynTree::FrameIndex& frameBaseIndex,
                             const std::string& name,
                             const iDynTree::Model& model)
{
    // get the link where the desired baseFrame is attached
    iDynTree::LinkIndex linkBaseIndex = model.getFrameLink(frameBaseIndex);

    // get the main frame of the link. In iDynTree the base frame has to be the main frame
    // of the link
    std::string baseFrameName = model.getLinkName(linkBaseIndex);

    m_baseFrames.insert(
        {name, {baseFrameName, model.getFrameTransform(frameBaseIndex).inverse()}});

    return true;
}
