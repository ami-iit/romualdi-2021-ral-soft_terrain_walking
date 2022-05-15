/**
 * @file WalkingModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iDynTree/Core/MatrixView.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iostream>
#include <manif/impl/so3/SO3.h>
#include <memory>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>


// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <SoftTerrainWalking/Module/Module.h>
#include <WalkingControllers/StdUtilities/Helper.h>

// Bipedal Locomotion Controller
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

iDynTree::Position desiredCoMPosition;
double desiredCoMHightZero;
iDynTree::Vector3 m_desiredCoMAcceleration;
double scaling = 1e5;


using namespace SoftTerrainWalking;

int indexVisualize = 0;

bool canAdvance{true};

bool leftWasInContact{true};
bool rightWasInContact{true};

iDynTree::VectorDynSize jointRegularization;

iDynTree::SpatialForceVector desiredMomentum;

inline manif::SO3d toManifRot(Eigen::Ref<const Eigen::Matrix3d> rotation)
{
    Eigen::Quaterniond quat = Eigen::Quaterniond(rotation);
    quat.normalize(); // SO3 constructor expects normalized quaternion
    return manif::SO3d(quat);
}

void WalkingModule::propagateTime()
{
    // propagate time
    m_time += m_dT;
}

bool WalkingModule::advanceReferenceSignals()
{
    // check if vector is not initialized
    if(m_leftTrajectory.empty()
       || m_rightTrajectory.empty()
       || m_leftInContact.empty()
       || m_rightInContact.empty()
       || m_DCMPositionDesired.empty()
       || m_DCMVelocityDesired.empty()
       || m_comHeightTrajectory.empty())
    {
        yError() << "[WalkingModule::advanceReferenceSignals] Cannot advance empty reference signals.";
        return false;
    }

    m_rightTrajectory.pop_front();
    m_rightTrajectory.push_back(m_rightTrajectory.back());

    m_leftTrajectory.pop_front();
    m_leftTrajectory.push_back(m_leftTrajectory.back());

    m_rightTwistTrajectory.pop_front();
    m_rightTwistTrajectory.push_back(m_rightTwistTrajectory.back());

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_rightAccelerationTrajectory.pop_front();
    m_rightAccelerationTrajectory.push_back(m_rightAccelerationTrajectory.back());

    m_leftAccelerationTrajectory.pop_front();
    m_leftAccelerationTrajectory.push_back(m_leftAccelerationTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_ZMPPositionDesired.pop_front();
    m_ZMPPositionDesired.push_back(m_ZMPPositionDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    m_weightInLeft.pop_front();
    m_weightInLeft.push_back(m_weightInLeft.back());

    m_weightInRight.pop_front();
    m_weightInRight.push_back(m_weightInRight.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if(!m_mergePoints.empty())
    {
        for(auto& mergePoint : m_mergePoints)
            mergePoint--;

        if(m_mergePoints[0] == 0)
            m_mergePoints.pop_front();
    }
    return true;
}

double WalkingModule::getPeriod()
{
    //  period of the module (seconds)
    return m_dT;
}

bool WalkingModule::setRobotModel(const yarp::os::Searchable& rf)
{
    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model",yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    yInfo() << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_axesList))
    {
        yError() << "[WalkingModule::setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder& rf)
{

    using namespace BipedalLocomotion::ParametersHandler;
    YarpImplementation::shared_ptr parametersHandler;
    parametersHandler = std::make_shared<YarpImplementation>(rf);

    // module name (used as prefix for opened ports)
    m_useOSQP = rf.check("use_osqp", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();
    m_useTorqueControl = rf.check("use_torque_control", yarp::os::Value(false)).asBool();

    m_axesList.resize(23);
    if(!parametersHandler->getParameter("joints_list", m_axesList))
    {
        yError() << "[WalkingModule::configure] Unable to find the joints_list";
        return false;
    }


    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    if(m_useTorqueControl && !iDynTree::parseRotationMatrix(generalOptions, "additional_rotation", m_additionalRotation))
    {
        yError() << "[WalkingModule::configure] Unable to set the additional rotation.";
        return false;
    }

    std::string name;
    if(!parametersHandler->getGroup("GENERAL").lock()->getParameter("name", name))
    {
        yError() << "[WalkingModule::configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());


    if(!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    using namespace BipedalLocomotion::ParametersHandler;
    YarpImplementation::shared_ptr configSimulator;
    configSimulator = std::make_shared<YarpImplementation>(rf.findGroup("SIMULATOR"));
    m_simulator = std::make_unique<SoftTerrainWalking::Simulator::Simulator>();
    if(!m_simulator->initialize(configSimulator, m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Unable to configure the simulator.";
        return false;
    }


    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    std::string desiredUnyciclePositionPortName = "/" + getName() + "/goal:i";
    if(!m_desiredUnyciclePositionPort.open(desiredUnyciclePositionPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << desiredUnyciclePositionPortName << " port.";
        return false;
    }

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryPlanner::TrajectoryGenerator>();
    yarp::os::Bottle& trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER");
    trajectoryPlannerOptions.append(generalOptions);
    if(!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WholeBodyControllersRigid::WalkingIK>();
    yarp::os::Bottle& inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if(!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(),
                               m_axesList))
    {
        yError() << "[WalkingModule::configure] Failed to configure the ik solver";
        return false;
    }


    yarp::os::Bottle& inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");
    inverseKinematicsQPSolverOptions.append(generalOptions);
    if(m_useOSQP)
        m_QPIKSolver = std::make_unique<WholeBodyControllersRigid::WalkingQPIK_osqp>();
    else
        m_QPIKSolver = std::make_unique<WholeBodyControllersRigid::WalkingQPIK_qpOASES>();

    // create fake bounds
    iDynTree::VectorDynSize dummyLimit(m_simulator->getActuatedDoFs());
    iDynTree::VectorDynSize dummyLimitNegative(m_simulator->getActuatedDoFs());
    for (int i = 0; i < m_simulator->getActuatedDoFs(); i++)
    {
        dummyLimit(i) = 10000;
        dummyLimitNegative(i) = -10000;
    }

    if(!m_QPIKSolver->initialize(inverseKinematicsQPSolverOptions,
                                 m_simulator->getActuatedDoFs(),
                                 dummyLimit,
                                 dummyLimit,
                                 dummyLimitNegative))
    {
        yError() << "[WalkingModule::configure] Failed to configure the QP-IK solver (qpOASES)";
        return false;
    }


    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle& forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if(!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the fk solver";
        return false;
    }

    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<TrajectoryPlanner::StableDCMModel>();
    if(!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the lipm.";
        return false;
    }

    // initialize the logger
    if(m_dumpData)
    {
        m_walkingLogger = std::make_unique<WalkingControllers::LoggerClient>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, getName()))
        {
            yError() << "[WalkingModule::configure] Unable to configure the logger.";
            return false;
        }
    }


    // create a floating base kinematics  system
    m_desiredFloatingBaseSystemKinematics = std::make_shared<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>();
    double dTSimulator;
    if(!configSimulator->getParameter("sampling_time", dTSimulator))
    {
        std::cerr << "Unable to find the sampling time of the simulator" << std::endl;
        return false;
    }

    m_desiredFloatingBaseSystemKinematicsIntegrator
        = std::make_unique<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>();
    m_desiredFloatingBaseSystemKinematicsIntegrator->setIntegrationStep(dTSimulator);

    m_visualizer = std::make_unique<SoftTerrainWalking::Simulator::Visualizer>();
    m_visualizer->addModel(m_loader.model(), "iCub");

    // load the model in iDynTree::KinDynComputations
    std::string modelPlane = "plane.urdf";
    std::string pathToModel
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(modelPlane);

    yInfo() << "[WalkingModule::setRobotModel] The plane is found in: " << pathToModel;

    // iDynTree::ModelLoader planeLoader;

    // // only the controlled joints are extracted from the URDF file
    // if(!planeLoader.loadModelFromFile(pathToModel))
    // {
    //     yError() << "[WalkingModule::setRobotModel] Error while loading the model from " << pathToModel;
    //     return false;
    // }


    m_visualizer->addPlane("plane");

    // Task based torque controller
    if(m_useTorqueControl)
    {
        m_momentumBasedController = std::make_unique<SoftTerrainWalking::WholeBodyControllers::MomentumBasedControl>(m_FKSolver->getKinDyn());
        yarp::os::Bottle momentumBasedTorqueControlOptions = rf.findGroup("TORQUE_CONTROL");
        momentumBasedTorqueControlOptions.append(generalOptions);
        YarpImplementation::shared_ptr config;
        config = std::make_shared<YarpImplementation>(momentumBasedTorqueControlOptions);
        YarpImplementation::weak_ptr configWeak = config;
        try
        {
            if(!m_momentumBasedController->initialize(configWeak))
            {
                return false;
            }
        }
        catch(const std::exception& e)
        {
            yError() << e.what();
            return false;
        }
    }

    yarp::os::Bottle configFootModelYarp = rf.findGroup("CONTACT_MODEL");
    auto configFootModel = std::make_shared<YarpImplementation>(configFootModelYarp);
    m_leftFootEstimator.model = std::make_unique<SoftTerrainWalking::ContactModels::ContinuousContactModel>();
    if(!m_leftFootEstimator.model->initialize(configFootModel))
    {
        yError() << "Unable to configure the left foot model";
        return false;
    }

    m_rightFootEstimator.model = std::make_unique<SoftTerrainWalking::ContactModels::ContinuousContactModel>();
    if(!m_rightFootEstimator.model->initialize(configFootModel))
    {
        yError() << "Unable to configure the right foot model";
        return false;
    }

    yarp::os::Bottle configFootEstimatorsYarp = rf.findGroup("RLS_ESTIMATOR");
    auto configFootEstimators = std::make_shared<YarpImplementation>(configFootEstimatorsYarp);

    m_leftFootEstimator.rls = std::make_unique<BipedalLocomotion::Estimators::RecursiveLeastSquare>();
    if(!m_leftFootEstimator.rls->initialize(configFootEstimators))
    {
        yError() << "Unable to configure the left foot estimator";
        return false;
    }
    m_leftFootEstimator.rls->setRegressorFunction([this]() -> Eigen::MatrixXd {
        return iDynTree::toEigen(m_leftFootEstimator.model->getRegressor());
    });

    m_rightFootEstimator.rls = std::make_unique<BipedalLocomotion::Estimators::RecursiveLeastSquare>();
    if(!m_rightFootEstimator.rls->initialize(configFootEstimators))
    {
        yError() << "Unable to configure the right foot estimator";
        return false;
    }
    m_rightFootEstimator.rls->setRegressorFunction([this]() -> Eigen::MatrixXd {
        return iDynTree::toEigen(m_rightFootEstimator.model->getRegressor());
    });


   // Task based torque control
    m_taskBasedTorqueControl = std::make_unique<WholeBodyControllersRigid::TaskBasedTorqueControlYARPInterface>(*m_FKSolver->getKinDyn());
    yarp::os::Bottle& taskBasedTorqueControlOptions = rf.findGroup("TASK_BASED_TORQUE_CONTROL");
    try
    {
        m_taskBasedTorqueControl->initialize(taskBasedTorqueControlOptions);
    }
    catch(const std::exception& e)
    {
        yError() << e.what();
        return false;
    }

    // time profiler
    m_profiler = std::make_unique<WalkingControllers::TimeProfiler>();
    m_profiler->setPeriod(round(0.1 / m_dT));

    if(m_useTorqueControl)
        m_profiler->addTimer("TORQUE_CONTROL");
    else
        m_profiler->addTimer("IK");

    m_profiler->addTimer("Total");

    // initialize some variables
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();

    // resize variables
    m_qDesired.resize(m_simulator->getActuatedDoFs());
    m_dqDesired.resize(m_simulator->getActuatedDoFs());

    yInfo() << "[WalkingModule::configure] Ready to play!";


    this->prepareRobot();
    this->startWalking();

    return true;
}

void WalkingModule::reset()
{
    m_trajectoryGenerator->reset();

    if(m_dumpData)
        m_walkingLogger->quit();
}

bool WalkingModule::close()
{
    if(m_dumpData)
        m_walkingLogger->quit();

    // close the ports
    m_rpcPort.close();
    m_desiredUnyciclePositionPort.close();


    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_QPIKSolver.reset(nullptr);
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);

    return true;
}

bool WalkingModule::solveQPIK(const std::unique_ptr<WholeBodyControllersRigid::WalkingQPIK>& solver,
                              const iDynTree::Position& desiredCoMPosition,
                              const iDynTree::Vector3& desiredCoMVelocity,
                              const iDynTree::Rotation& desiredNeckOrientation,
                              iDynTree::VectorDynSize& output)
{
    bool ok = true;
    double threshold = 0.001;
    bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
    solver->setPhase(stancePhase);

    auto [basePosition, baseRotation, jointPositions]  = m_desiredFloatingBaseSystemKinematicsIntegrator->getSolution();

    iDynTree::VectorDynSize jointPositionsiDyn(jointPositions.size());
    iDynTree::toEigen(jointPositionsiDyn) = jointPositions;
    ok &= solver->setRobotState(jointPositionsiDyn,
                                m_FKSolver->getLeftFootToWorldTransform(),
                                m_FKSolver->getRightFootToWorldTransform(),
                                m_FKSolver->getLeftHandToWorldTransform(),
                                m_FKSolver->getRightHandToWorldTransform(),
                                m_FKSolver->getNeckOrientation(),
                                m_FKSolver->getCoMPosition());

    solver->setDesiredNeckOrientation(desiredNeckOrientation.inverse());

    solver->setDesiredFeetTransformation(m_leftTrajectory.front(),
                                         m_rightTrajectory.front());

    solver->setDesiredFeetTwist(m_leftTwistTrajectory.front(),
                                m_rightTwistTrajectory.front());

    solver->setDesiredCoMVelocity(desiredCoMVelocity);
    solver->setDesiredCoMPosition(desiredCoMPosition);

    // set jacobians
    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_simulator->getActuatedDoFs() + 6);
    comJacobian.resize(3, m_simulator->getActuatedDoFs() + 6);

    ok &= m_FKSolver->getLeftFootJacobian(jacobian);
    ok &= solver->setLeftFootJacobian(jacobian);

    ok &= m_FKSolver->getRightFootJacobian(jacobian);
    ok &= solver->setRightFootJacobian(jacobian);

    ok &= m_FKSolver->getNeckJacobian(jacobian);
    ok &= solver->setNeckJacobian(jacobian);

    ok &= m_FKSolver->getCoMJacobian(comJacobian);
    solver->setCoMJacobian(comJacobian);

    ok &= m_FKSolver->getLeftHandJacobian(jacobian);
    ok &= solver->setLeftHandJacobian(jacobian);

    ok &= m_FKSolver->getRightHandJacobian(jacobian);
    ok &= solver->setRightHandJacobian(jacobian);

    if(!ok)
    {
        yError() << "[WalkingModule::solveQPIK] Error while setting the jacobians.";
        return false;
    }

    if(!solver->solve())
    {
        yError() << "[WalkingModule::solveQPIK] Unable to solve the QP-IK problem.";
        return false;
    }

    output = solver->getDesiredJointVelocities();

    return true;
}

bool WalkingModule::estimateContactParameters()
{
    double area = 0.2 * 0.1;

    if(m_leftInContact.front())
    {
        // TODO do it better.
        m_leftFootEstimator.model->setState(m_FKSolver->getLeftFootVelocity(), m_FKSolver->getLeftFootToWorldTransform());
        m_leftFootEstimator.model->setNullForceTransform(m_leftFootEstimator.nullForceTransform);

        iDynTree::VectorDynSize leftWrench(6);
        iDynTree::toEigen(leftWrench) = iDynTree::toEigen(m_simulator->leftWrench()) / (area * scaling);
        m_leftFootEstimator.rls->setMeasurements(iDynTree::toEigen(leftWrench));


        if(!m_leftFootEstimator.rls->advance())
        {
            yError() << "[WalkingModule::estimateContactParameters] Unable to estimate the left foot parameters";
            return false;
        }
    }

    if(m_rightInContact.front())
    {
        // TODO do it better.
        m_rightFootEstimator.model->setState(m_FKSolver->getRightFootVelocity(), m_FKSolver->getRightFootToWorldTransform());
        m_rightFootEstimator.model->setNullForceTransform(m_rightFootEstimator.nullForceTransform);

        iDynTree::VectorDynSize rightWrench(6);
        iDynTree::toEigen(rightWrench) = iDynTree::toEigen(m_simulator->rightWrench()) / (area * scaling);
        m_rightFootEstimator.rls->setMeasurements(iDynTree::toEigen(rightWrench));

        if(!m_rightFootEstimator.rls->advance())
        {
            yError() << "[WalkingModule::estimateContactParameters] Unable to estimate the right foot parameters";
            return false;
        }
    }

    return true;
}

bool WalkingModule::solveMomentumBasedWithCompliantContacts(const iDynTree::Position &desiredCoMPosition,
                                                            const iDynTree::Vector3 &desiredCoMVelocity,
                                                            const iDynTree::Vector3 &desiredCoMAcceleration,
                                                            const iDynTree::Vector3 &desiredCoMJerk,
                                                            const iDynTree::Rotation &desiredNeckOrientation)
{
    double robotMass = m_FKSolver->getKinDyn()->model().getTotalMass();
    m_momentumBasedController->setFeetState(m_leftInContact.front(), m_rightInContact.front());
    double kLeft = m_leftFootEstimator.rls->getOutput().expectedValue(0) * scaling;
    double bLeft = m_leftFootEstimator.rls->getOutput().expectedValue(1) * scaling;
    m_momentumBasedController->setContactParameters("left_foot", kLeft, bLeft);

    m_leftFootEstimator.model->springCoeff() = kLeft;
    m_leftFootEstimator.model->damperCoeff() = bLeft;


    double kRight = m_rightFootEstimator.rls->getOutput().expectedValue(0) * scaling;
    double bRight = m_rightFootEstimator.rls->getOutput().expectedValue(1) * scaling;
    m_rightFootEstimator.model->springCoeff() = kRight;
    m_rightFootEstimator.model->damperCoeff() = bRight;

    m_momentumBasedController->setContactParameters("right_foot", kRight, bRight);

    m_momentumBasedController->setFootUpperBoundNormalForce("right_foot", m_weightInRight[1] * robotMass * 10);
    m_momentumBasedController->setFootUpperBoundNormalForce("left_foot", m_weightInLeft[1] * robotMass * 10);

    // iDynTree::toEigen(m_FKSolver->getCentroidalMomentumJacobian()) * iDynTree::toEigen(m_QPIKSolver->getSolution())

    iDynTree::SpatialForceVector momentumDerivative, dummySpatial, momentumDoubleDerivative;
    dummySpatial.zero();
    momentumDerivative.zero();
    momentumDoubleDerivative.zero();
    desiredMomentum.getLinearVec3()(0) = desiredCoMVelocity(0) * robotMass;
    desiredMomentum.getLinearVec3()(1) = desiredCoMVelocity(1) * robotMass;
    desiredMomentum.getLinearVec3()(2) = desiredCoMVelocity(2) * robotMass;

    momentumDerivative.getLinearVec3()(0) = desiredCoMAcceleration(0) * robotMass;
    momentumDerivative.getLinearVec3()(1) = desiredCoMAcceleration(1) * robotMass;
    momentumDerivative.getLinearVec3()(2) = desiredCoMAcceleration(2) * robotMass;


    momentumDoubleDerivative.getLinearVec3()(0) = desiredCoMJerk(0) * robotMass;
    momentumDoubleDerivative.getLinearVec3()(1) = desiredCoMJerk(1) * robotMass;
    momentumDoubleDerivative.getLinearVec3()(2) = desiredCoMJerk(2) * robotMass;


    iDynTree::Vector3 dummy;
    dummy.zero();

    m_momentumBasedController->setCentroidalMomentumReference(
        momentumDoubleDerivative, momentumDerivative, desiredMomentum, desiredCoMPosition);

    iDynTree::SpatialAcc dummySpatialAcc;
    dummySpatialAcc.zero();

    // torso orientation
    m_momentumBasedController->setRotationReference(dummy, dummy, desiredNeckOrientation * m_additionalRotation, "torso");

    m_momentumBasedController->setRotationReference(dummy, dummy, desiredNeckOrientation * iDynTree::Rotation::RotZ(M_PI), "root");
    m_momentumBasedController->setTransformationReference(m_leftAccelerationTrajectory.front(),
                                                          m_leftTwistTrajectory.front(),
                                                          m_leftTrajectory.front(), "left_foot");

    m_momentumBasedController->setTransformationReference(m_rightAccelerationTrajectory.front(),
                                                          m_rightTwistTrajectory.front(),
                                                          m_rightTrajectory.front(), "right_foot");


    m_momentumBasedController->setMeasuredContactWrench({{"left_foot", m_leftFootEstimator.model->getContactWrench()},
            {"right_foot", m_rightFootEstimator.model->getContactWrench()}});


    // m_momentumBasedController->setMeasuredContactWrench({{"left_foot", m_simulator->leftWrench()},
    //         {"right_foot", m_simulator->rightWrench()}});

    iDynTree::VectorDynSize dummyJoint(m_simulator->getActuatedDoFs());
    dummyJoint.zero();
    m_momentumBasedController->setRegularizationReference(
        dummyJoint, m_dqDesired, m_qDesired, "joint_accelerations");

    m_momentumBasedController->setJointState(m_simulator->jointVelocities(), m_simulator->jointPositions());

    return m_momentumBasedController->solve();
}

void WalkingModule::solveTaskBasedInverseDynamics(const iDynTree::Position& desiredCoMPosition,
                                                  const iDynTree::Vector3& desiredCoMVelocity,
                                                  const iDynTree::Vector3& desiredCoMAcceleration,
                                                  const iDynTree::Rotation& desiredNeckOrientation)
{
    iDynTree::Vector3 dummy;
    dummy.zero();

    // set feet status
    m_taskBasedTorqueControl->setContactStateCartesianElement(false, "left_foot");
    m_taskBasedTorqueControl->setContactStateCartesianElement(false, "right_foot");

    m_taskBasedTorqueControl->setContactStateWrenchFeasibilityElement(m_leftInContact.front(), "left_foot");
    m_taskBasedTorqueControl->setContactStateWrenchFeasibilityElement(m_rightInContact.front(), "right_foot");

    m_taskBasedTorqueControl->setWeight(m_weightInLeft.front(), "left_foot");
    m_taskBasedTorqueControl->setWeight(m_weightInRight.front(), "right_foot");

    // set CoM trajectory
    m_taskBasedTorqueControl->setDesiredCartesianTrajectory(desiredCoMAcceleration, desiredCoMVelocity,
                                                            desiredCoMPosition, "CoM");
    // set desired angular momentum
    m_taskBasedTorqueControl->setDesiredAngularMomentum(dummy, dummy);

    // set feet trajectories
    m_taskBasedTorqueControl->setDesiredCartesianTrajectory(m_leftAccelerationTrajectory.front(), m_leftTwistTrajectory.front(),
                                                            m_leftTrajectory.front() , "left_foot");

    m_taskBasedTorqueControl->setDesiredCartesianTrajectory(m_rightAccelerationTrajectory.front(), m_rightTwistTrajectory.front(),
                                                            m_rightTrajectory.front() , "right_foot");

    // regularization term
    iDynTree::VectorDynSize dummyJoint(m_simulator->getActuatedDoFs());
    dummyJoint.zero();
    m_taskBasedTorqueControl->setDesiredRegularizationTrajectory(dummyJoint, dummyJoint, m_qDesired, "joint_accelerations");
    m_taskBasedTorqueControl->setJointState(m_simulator->jointVelocities(), m_simulator->jointPositions());

    // torso orientation
    m_taskBasedTorqueControl->setDesiredCartesianTrajectory(dummy, dummy, desiredNeckOrientation, "torso");

    m_taskBasedTorqueControl->solve();

    return;
}

bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState == WalkingFSM::Walking)
    {
        bool resetTrajectory = false;

        m_profiler->setInitTime("Total");

        // check desired planner input
        yarp::sig::Vector* desiredUnicyclePosition = nullptr;
        desiredUnicyclePosition = m_desiredUnyciclePositionPort.read(false);
        if(desiredUnicyclePosition != nullptr)
            if(!setPlannerInput((*desiredUnicyclePosition)(0), (*desiredUnicyclePosition)(1)))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the planner input";
                return false;
            }

        // if a new trajectory is required check if its the time to evaluate the new trajectory or
        // the time to attach new one
        if(m_newTrajectoryRequired)
        {
            // when we are near to the merge point the new trajectory is evaluated
            if(m_newTrajectoryMergeCounter == 20)
            {

                double initTimeTrajectory;
                initTimeTrajectory = m_time + m_newTrajectoryMergeCounter * m_dT;

                iDynTree::Transform measuredTransform = m_isLeftFixedFrame.front() ?
                    m_rightTrajectory[m_newTrajectoryMergeCounter] :
                    m_leftTrajectory[m_newTrajectoryMergeCounter];

                // ask for a new trajectory
                if(!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                       measuredTransform, m_newTrajectoryMergeCounter,
                                       m_desiredPosition))
                {
                    yError() << "[WalkingModule::updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if(m_newTrajectoryMergeCounter == 2)
            {
                if(!updateTrajectories(m_newTrajectoryMergeCounter))
                {
                    yError() << "[WalkingModule::updateModule] Error while updating trajectories. They were not computed yet.";
                    canAdvance = false;
                }
                if(canAdvance)
                {
                    m_newTrajectoryRequired = false;
                    resetTrajectory = true;
                }
            }
            if(canAdvance)
                m_newTrajectoryMergeCounter--;
        }


        if(!updateFKSolver())
        {
            yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
            return false;
        }

        // notice done after update the FK
        if(!leftWasInContact)
        {
            if(m_leftInContact.front())
            {
                leftWasInContact = m_leftInContact.front();

                // set the null force transform in the simulator
                m_simulator->setLeftFootState(leftWasInContact);
                m_simulator->setLeftFootNullForceTransform(m_FKSolver->getLeftFootToWorldTransform());

                // set the null force transform in the controller
                m_momentumBasedController->setContactState(m_FKSolver->getLeftFootToWorldTransform(),
                                                           "left_foot");


                std::cerr << "null force transform set left" << m_FKSolver->getLeftFootToWorldTransform().toString() << std::endl;

                // set null force transform for the estimator
                m_leftFootEstimator.nullForceTransform = m_FKSolver->getLeftFootToWorldTransform();

                // reset estimator covariance
                m_leftFootEstimator.rls->resetStateCovarianceToInitialState();
            }
        }
        else
        {
            if(!m_leftInContact.front())
            {
                leftWasInContact = m_leftInContact.front();
                m_simulator->setLeftFootState(leftWasInContact);
            }

        }

        if(!rightWasInContact)
        {
            if(m_rightInContact.front())
            {
                rightWasInContact = m_rightInContact.front();

                m_simulator->setRightFootState(rightWasInContact);

                // set the null force transform in the simulator
                m_simulator->setRightFootNullForceTransform(m_FKSolver->getRightFootToWorldTransform());

                // set the null force transform in the controller
                m_momentumBasedController->setContactState(m_FKSolver->getRightFootToWorldTransform(),
                                                           "right_foot");

                // set null force transform for the estimator
                m_rightFootEstimator.nullForceTransform = m_FKSolver->getRightFootToWorldTransform();

                std::cerr << "null force transform set right" << m_FKSolver->getRightFootToWorldTransform().toString() << std::endl;


                // reset estimator covariance
                m_rightFootEstimator.rls->resetStateCovarianceToInitialState();
            }
        }
        else
        {
            if(!m_rightInContact.front())
            {
                rightWasInContact = m_rightInContact.front();
                m_simulator->setRightFootState(rightWasInContact);
            }

        }



        m_stableDCMModel->setDCMPosition(m_DCMPositionDesired.front());
        m_stableDCMModel->setZMPPosition(m_ZMPPositionDesired.front());
        if(!m_stableDCMModel->integrateModel())
        {
            yError() << "[WalkingModule::updateModule] Unable to propagate the 3D-LIPM.";
            return false;
        }

        iDynTree::Vector3 desiredCoMVelocity;
        iDynTree::Vector3 desiredCoMJerk;

        desiredCoMPosition(0) = m_stableDCMModel->getCoMPosition()(0);
        desiredCoMPosition(1) = m_stableDCMModel->getCoMPosition()(1);

        desiredCoMVelocity(0) = m_stableDCMModel->getCoMVelocity()(0);
        desiredCoMVelocity(1) = m_stableDCMModel->getCoMVelocity()(1);
        desiredCoMVelocity(2) = 0;

        desiredCoMJerk.zero();
        desiredCoMJerk(0) = (m_stableDCMModel->getCoMAcceleration()(0)  - m_desiredCoMAcceleration(0)) / m_dT;
        desiredCoMJerk(1) = (m_stableDCMModel->getCoMAcceleration()(1)  - m_desiredCoMAcceleration(1)) / m_dT;

        m_desiredCoMAcceleration(0) = m_stableDCMModel->getCoMAcceleration()(0);
        m_desiredCoMAcceleration(1) = m_stableDCMModel->getCoMAcceleration()(1);

        // evaluate desired neck transformation
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        yawRotation = iDynTree::Rotation::RotZ(meanYaw);
        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;


        // integrate dq because velocity control mode seems not available
        yarp::sig::Vector bufferVelocity(m_simulator->getActuatedDoFs());
        yarp::sig::Vector bufferPosition(m_simulator->getActuatedDoFs());

        const auto& [basePosition, baseRotation, jointPositions]  = m_desiredFloatingBaseSystemKinematicsIntegrator->getSolution();
        iDynTree::Twist zeroTwist;
        zeroTwist.zero();
        iDynTree::VectorDynSize dummyJoint(m_simulator->getActuatedDoFs());
        dummyJoint.zero();

        // the IK-QP does not use velocities

        iDynTree::Rotation tempRot;
        iDynTree::toEigen(tempRot) = baseRotation.rotation();

        m_FKSolver->evaluateWorldToBaseTransformation(iDynTree::Transform(tempRot,
                                                                          iDynTree::make_span(
                                                                              basePosition)),
                                                      zeroTwist);
        if (!m_FKSolver->setInternalRobotState(iDynTree::make_span(jointPositions), dummyJoint))
        {
            yError() << "[WalkingModule::updateFKSolver] Unable to set the robot state.";
            return false;
        }

        iDynTree::Position desredCoMPositionIK = desiredCoMPosition;
        desredCoMPositionIK(2) = desiredCoMHightZero;

        if(!solveQPIK(m_QPIKSolver, desredCoMPositionIK,
                      desiredCoMVelocity,
                      yawRotation, m_dqDesired))
        {
            yError() << "[WalkingModule::updateModule] Unable to solve the QP problem with osqp.";
            return false;
        }

        iDynTree::toEigen(desiredMomentum.getAngularVec3()) = (iDynTree::toEigen(m_FKSolver->getCentroidalMomentumJacobian())
                                                               * iDynTree::toEigen(m_QPIKSolver->getSolution())).tail<3>();

        updateFKSolver();

        if(!estimateContactParameters())
        {
            yError() << "Unable to estimate the contact parameters";
            return false;
        }

        m_profiler->setInitTime("TORQUE_CONTROL");

        try
        {
            // double position, velocity, acceleration;
            // position = m_weightToComHeightSpline.evaluatePoint(m_weightInLeft.front(), velocity, acceleration);
            // desiredCoMPosition(2) = desiredCoMHightZero + position;
            // desiredCoMVelocity(2) = velocity * (m_weightInLeft[1] - m_weightInLeft[0]) / m_dT;

            // double acc = acceleration * (m_weightInLeft[1] - m_weightInLeft[0]) / m_dT +
            //     velocity * (m_weightInLeft[2] - 2 * m_weightInLeft[1] + m_weightInLeft[0]) / m_dT;

            // desiredCoMJerk(2) = (acc  - m_desiredCoMAcceleration(2)) / m_dT;

            // desiredCoMJerk.zero();
            // m_desiredCoMAcceleration(2) = acc;

            // std::cerr << desiredCoMJerk.toString() << std::endl;

            // solveTaskBasedInverseDynamics(desredCoMPositionIK, desiredCoMVelocity,
            //                               m_desiredCoMAcceleration,
            //                               yawRotation.inverse() * m_additionalRotation);

            if(!solveMomentumBasedWithCompliantContacts(desiredCoMPosition, desiredCoMVelocity,
                                                        m_desiredCoMAcceleration,
                                                        desiredCoMJerk,
                                                        yawRotation.inverse())) //* iDynTree::Rotation::RotZ(M_PI))) // * m_additionalRotation))
            {
                yError() << "Unable to solve the problem";
                return false;
            }
        }
        catch(const std::exception& e)
        {
            yError() << e.what();
            return false;
        }



        if (!m_simulator->setTorqueReferences(
                m_momentumBasedController->getDesiredTorques())) {
          yError() << "[WalkingModule::updateModule] Unable to set the desired "
                      "acceleration";
          return false;
        }
        // if (!m_simulator->setTorqueReferences(
        //         m_taskBasedTorqueControl->getDesiredTorques())) {
        //   yError() << "[WalkingModule::updateModule] Unable to set the desired "
        //               "acceleration";
        //   return false;
        // }


        m_profiler->setEndTime("TORQUE_CONTROL");

        Eigen::Matrix<double, 6, 1> baseTwistDesired;
        Eigen::VectorXd jointVelocityDesired(m_simulator->getActuatedDoFs());

        baseTwistDesired = iDynTree::toEigen(m_QPIKSolver->getSolution()).head<6>();
        jointVelocityDesired = iDynTree::toEigen(m_QPIKSolver->getSolution()).tail(m_simulator->getActuatedDoFs());
        m_desiredFloatingBaseSystemKinematics->setControlInput({baseTwistDesired, jointVelocityDesired});

        m_desiredFloatingBaseSystemKinematicsIntegrator->integrate(0, m_dT);

        // advance simulator
        if(!m_simulator->advance(m_dT))
        {
            return false;
        }

        if (indexVisualize == 0)
        {
            m_visualizer->setCameraTarget(iDynTree::make_span(
                std::get<0>(m_desiredFloatingBaseSystemKinematicsIntegrator->getSolution())));
            m_visualizer->visualizeState(m_simulator->baseTransform(),
                                         m_simulator->jointPositions(),
                                         {{m_FKSolver->getLeftFootToWorldTransform(), m_simulator->leftWrench()},
                                             {m_FKSolver->getRightFootToWorldTransform(), m_simulator->rightWrench()}});
            if(!m_visualizer->saveFrame())
            {
                return false;
            }

        }

        indexVisualize++;

        if(indexVisualize == static_cast<int>(0.02 / m_dT))
            indexVisualize = 0;

        iDynTree::Vector2 covLeft;
        auto covLeftMatrix = m_leftFootEstimator.rls->getOutput().covariance;
        covLeft(0) = covLeftMatrix(0,0);
        covLeft(1) = covLeftMatrix(1,1);

        iDynTree::Vector2 covRight;
        auto covRightMatrix = m_rightFootEstimator.rls->getOutput().covariance;
        covRight(0) = covRightMatrix(0,0);
        covRight(1) = covRightMatrix(1,1);


        if (m_dumpData)
          m_walkingLogger->sendData(
              m_FKSolver->getCoMPosition(), desiredCoMPosition,
              m_FKSolver->getLeftFootToWorldTransform().getPosition(),
              m_FKSolver->getLeftFootToWorldTransform().getRotation().asRPY(),
              m_FKSolver->getRightFootToWorldTransform().getPosition(),
              m_FKSolver->getRightFootToWorldTransform().getRotation().asRPY(),
              m_leftTrajectory.front().getPosition(), m_leftTrajectory.front().getRotation().asRPY(),
              m_rightTrajectory.front().getPosition(), m_rightTrajectory.front().getRotation().asRPY(),
              m_FKSolver->getLeftFootVelocity(),
              // m_momentumBasedController->getLeftFootWrenchRateOfChange(),
              // m_momentumBasedController->getRightFootWrenchRateOfChange(),
              m_simulator->leftWrench(), m_simulator->rightWrench(),
              desiredMomentum, m_FKSolver->getKinDyn()->getCentroidalTotalMomentum(),
              m_leftFootEstimator.rls->getOutput().expectedValue, covLeft,
              m_rightFootEstimator.rls->getOutput().expectedValue, covRight,
              yarp::sig::Vector(1,m_simulator->leftContactModel()->springCoeff()),
              yarp::sig::Vector(1,m_simulator->leftContactModel()->damperCoeff()),
              yarp::sig::Vector(1,m_simulator->rightContactModel()->springCoeff()),
              yarp::sig::Vector(1,m_simulator->rightContactModel()->damperCoeff()));

        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();
        if(canAdvance)
        {
            advanceReferenceSignals();
            propagateTime();
        }
        canAdvance = true;

        // experiment ended
        if (m_time >= 25)
            return false;
    }

    return true;
}


bool WalkingModule::prepareRobot()
{
    if(m_robotState != WalkingFSM::Configured && m_robotState != WalkingFSM::Stopped)
    {
        yError() << "[WalkingModule::prepareRobot] The robot can be prepared only at the "
                 << "beginning or when the controller is stopped.";
        return false;
    }

    // evaluate the first trajectory. The robot does not move! So the first trajectory
    if(!generateFirstTrajectories())
    {
        yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
        return false;
    }


    if(m_IKSolver->usingAdditionalRotationTarget())
    {
        // get the yow angle of both feet
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        // evaluate the mean of the angles
        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        // it is important to notice that the inertial frames rotate with the robot
        yawRotation = iDynTree::Rotation::RotZ(meanYaw);

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[WalkingModule::prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }

    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0);
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();


    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                              desiredCoMPosition, m_qDesired))
    {
        yError() << "[WalkingModule::prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }
    m_dqDesired.zero();

    if(!m_simulator->reset(m_qDesired, m_leftTrajectory.front(), m_rightTrajectory.front()))
    {
        yError() << "[WalkingModule::prepareRobot] Unable to reset the simulator.";
        return false;
    }

    manif::SO3d tempRotation
        = toManifRot(iDynTree::toEigen(m_simulator->baseTransform().getRotation()));
    m_desiredFloatingBaseSystemKinematics->setState(
        {iDynTree::toEigen(m_simulator->baseTransform().getPosition()),
         tempRotation,
         iDynTree::toEigen(m_simulator->jointPositions())});

    m_desiredFloatingBaseSystemKinematicsIntegrator->setDynamicalSystem(m_desiredFloatingBaseSystemKinematics);

    // m_walkingZMPController->reset(m_DCMPositionDesired.front());
    m_stableDCMModel->reset(m_DCMPositionDesired.front());

    updateFKSolver();
    desiredCoMPosition = m_FKSolver->getCoMPosition();
    yarp::sig::Vector buffer(m_qDesired.size());
    iDynTree::toYarp(m_qDesired, buffer);
    // instantiate Integrator object

    m_QPIKSolver->setDesiredJointPosition(m_qDesired);

    yarp::sig::Matrix jointLimits(m_simulator->getActuatedDoFs(), 2);
    for(int i = 0; i < m_simulator->getActuatedDoFs(); i++)
    {
        jointLimits(i, 0) = -10000;
        jointLimits(i, 1) = 10000;
    }
    m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer, jointLimits);

    jointRegularization = m_qDesired;

    std::cerr << "desiredCoMPosition " << desiredCoMPosition.toString() << std::endl;
    desiredCoMHightZero = desiredCoMPosition(2);
    m_desiredCoMAcceleration.zero();

    // set null force transform for the estimator
    m_leftFootEstimator.nullForceTransform = m_leftTrajectory.front();
    m_rightFootEstimator.nullForceTransform = m_rightTrajectory.front();

    // create spline
    iDynTree::VectorDynSize weightAnchor(3);
    weightAnchor(0) = 0;
    weightAnchor(1) = 0.5;
    weightAnchor(2) = 1;

    iDynTree::VectorDynSize comHeightDisplacementAnchor(3);
    comHeightDisplacementAnchor(0) = m_FKSolver->getLeftFootToWorldTransform().getPosition()(2);
    comHeightDisplacementAnchor(1) = 0;
    comHeightDisplacementAnchor(2) = m_FKSolver->getLeftFootToWorldTransform().getPosition()(2);

    m_weightToComHeightSpline.setData(weightAnchor, comHeightDisplacementAnchor);
    m_weightToComHeightSpline.setFinalConditions(0, 0);
    m_weightToComHeightSpline.setInitialConditions(0, 0);


    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_robotState = WalkingFSM::Prepared;
    }

    m_visualizer->setCameraTarget(iDynTree::make_span(
        std::get<0>(m_desiredFloatingBaseSystemKinematicsIntegrator->getSolution())));
    m_visualizer->visualizeState(m_simulator->baseTransform(),
                                 m_simulator->jointPositions(),
                                 {{m_FKSolver->getLeftFootToWorldTransform(),
                                   m_simulator->leftWrench()},
                                  {m_FKSolver->getRightFootToWorldTransform(),
                                   m_simulator->rightWrench()}});

    return true;
}

bool WalkingModule::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::generateFirstTrajectories(const iDynTree::Position& initialPosition)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(initialPosition))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                       const iDynTree::Transform& measuredTransform,
                                       const size_t& mergePoint, const iDynTree::Vector2& desiredPosition)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if(mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[WalkingModule::askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }

    if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                  m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                  measuredTransform, desiredPosition))
    {
        yError() << "[WalkingModule::askNewTrajectories] Unable to update the trajectory.";
        return false;
    }
    return true;
}

bool WalkingModule::updateTrajectories(const size_t& mergePoint)
{
    if(!(m_trajectoryGenerator->isTrajectoryComputed()))
    {
        yError() << "[updateTrajectories] The trajectory is not computed.";
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<iDynTree::SpatialAcc> leftAccelerationTrajectory;
    std::vector<iDynTree::SpatialAcc> rightAccelerationTrajectory;
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<iDynTree::Vector2> ZMPPositionDesired;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;

    std::vector<double> weightInLeft;
    std::vector<double> weightInRight;

    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);

    // get ZMP position
    m_trajectoryGenerator->getZMPPositionTrajectory(ZMPPositionDesired);

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetAcceleration(leftAccelerationTrajectory, rightAccelerationTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // get weight percentage
    m_trajectoryGenerator->getWeightPercentage(weightInLeft, weightInRight);

    // append vectors to deques
    WalkingControllers::StdUtilities::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(leftAccelerationTrajectory, m_leftAccelerationTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(rightAccelerationTrajectory, m_rightAccelerationTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    WalkingControllers::StdUtilities::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(ZMPPositionDesired, m_ZMPPositionDesired, mergePoint);

    WalkingControllers::StdUtilities::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    WalkingControllers::StdUtilities::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    WalkingControllers::StdUtilities::appendVectorToDeque(weightInLeft, m_weightInLeft, mergePoint);
    WalkingControllers::StdUtilities::appendVectorToDeque(weightInRight, m_weightInRight, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    m_mergePoints.pop_front();

    return true;
}

bool WalkingModule::updateFKSolver()
{
    m_FKSolver->evaluateWorldToBaseTransformation(m_simulator->baseTransform(), m_simulator->baseVelocity());
    if(!m_FKSolver->setInternalRobotState(m_simulator->jointPositions(),
                                          m_simulator->jointVelocities()))
    {
        yError() << "[WalkingModule::updateFKSolver] Unable to set the robot state.";
        return false;
    }

    return true;
}

bool WalkingModule::startWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Prepared && m_robotState != WalkingFSM::Paused)
    {
        yError() << "[WalkingModule::startWalking] Unable to start walking if the robot is not prepared or paused.";
        return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record",
                    "com_x", "com_y", "com_z",
                    "com_des_x", "com_des_y", "com_des_z",
                    "lf_x", "lf_y", "lf_z",
                    "lf_roll", "lf_pitch", "lf_yaw",
                    "rf_x", "rf_y", "rf_z",
                    "rf_roll", "rf_pitch", "rf_yaw",
                    "lf_des_x", "lf_des_y", "lf_des_z",
                    "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                    "rf_des_x", "rf_des_y", "rf_des_z",
                    "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                    "lf_dx","lf_dy", "lf_dz",
                    "lf_wx","lf_wy", "lf_wz",
                    // "lf_force_rate_x", "lf_force_rate_y", "lf_force_rate_z",
                    // "lf_torque_rate_x", "lf_torque_rate_y", "lf_torque_rate_z",
                    // "rf_force_rate_x", "rf_force_rate_y", "rf_force_rate_z",
                    // "rf_torque_rate_x", "rf_torque_rate_y", "rf_torque_rate_z",
                    "lf_force_x", "lf_force_y", "lf_force_z",
                    "lf_torque_x", "lf_torque_y", "lf_torque_z",
                    "rf_force_x", "rf_force_y", "rf_force_z",
                    "rf_torque_x", "rf_torque_y", "rf_torque_z",
                    "linear_momentum_des_x", "linear_momentum_des_y", "linear_momentum_des_z",
                    "angular_momentum_des_x", "angular_momentum_des_y", "angular_momentum_des_z",
                    "linear_momentum_x", "linear_momentum_y", "linear_momentum_z",
                    "angular_momentum_x", "angular_momentum_y", "angular_momentum_z",
                    "k_lf", "b_lf", "k_cov_lf", "b_cov_lf", "k_rf", "b_rf", "k_cov_rf", "b_cov_rf",
                    "k_lf_real", "b_lf_real", "k_rf_real", "b_rf_real"
                    });
    }

    // TODO
    // // if the robot was only prepared the filters has to be reset

    m_robotState = WalkingFSM::Walking;

    return true;
}

bool WalkingModule::setPlannerInput(double x, double y)
{
    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if(m_mergePoints.empty())
    {
        if(!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[WalkingModule::setPlannerInput] The trajectory has already finished but the system is not in double support.";
            return false;
        }

        if(m_newTrajectoryRequired)
            return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged after x cycles
        m_newTrajectoryMergeCounter = 20;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        if(m_mergePoints.front() > 20)
            m_newTrajectoryMergeCounter = m_mergePoints.front();
        else if(m_mergePoints.size() > 1)
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = m_mergePoints[1];
        }
        else
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = 20;
        }
    }

    m_desiredPosition(0) = x;
    m_desiredPosition(1) = y;

    m_newTrajectoryRequired = true;

    return true;
}
