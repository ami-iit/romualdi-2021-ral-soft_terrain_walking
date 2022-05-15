/**
 * @file WalkingModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_MODULE_HPP
#define WALKING_MODULE_HPP

// std
#include <deque>
#include <memory>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RpcClient.h>

// iDynTree
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <SoftTerrainWalking/TrajectoryPlanner/StableDCMModel.h>
#include <SoftTerrainWalking/TrajectoryPlanner/TrajectoryGenerator.h>

#include <SoftTerrainWalking/WholeBodyControllersRigid/InverseKinematics.h>
#include <SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics.h>
#include <SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics_osqp.h>
#include <SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics_qpOASES.h>
#include <SoftTerrainWalking/WholeBodyControllersRigid/TaskBasedTorqueControl.h>

#include <SoftTerrainWalking/Simulator/Simulator.h>
#include <SoftTerrainWalking/Simulator/Visualizer.h>
#include <SoftTerrainWalking/WholeBodyControllers/MomentumBasedControl.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

#include <BipedalLocomotion/Estimators/RecursiveLeastSquare.h>
#include <SoftTerrainWalking/ContactModels/ContinuousContactModel.h>

#include <SoftTerrainWalking/KinDynWrapper/Wrapper.h>

#include <WalkingControllers/LoggerClient/LoggerClient.h>

#include <WalkingControllers/TimeProfiler/TimeProfiler.h>

// iCub-ctrl
#include <iCub/ctrl/filters.h>

#include <thrifts/SoftTerrainWalkingCommands.h>

namespace SoftTerrainWalking
{

/**
 * RFModule of the Walking controller
 */
class WalkingModule : public yarp::os::RFModule, public SoftTerrainWalkingCommands
{
    enum class WalkingFSM
    {
        Idle,
        Configured,
        Preparing,
        Prepared,
        Walking,
        WalkingPrecursor,
        Paused,
        Stopped
    };
    WalkingFSM m_robotState{WalkingFSM::Idle}; /**< State  of the WalkingFSM. */

    double m_dT; /**< RFModule period. */
    double m_time; /**< Current time. */
    std::string m_robot; /**< Robot name. */

    bool m_useOSQP; /**< True if osqp is used to QP-IK problem. */
    bool m_dumpData; /**< True if data are saved. */
    bool m_useTorqueControl; /**< True if the torque control is used. */

    std::vector<std::string> m_axesList;

    // std::unique_ptr<RobotInterface> m_robotControlHelper; /**< Robot control helper. */
    std::unique_ptr<TrajectoryPlanner::TrajectoryGenerator> m_trajectoryGenerator; /**< Pointer to
                                                                   the trajectory
                                                                   generator object. */
    std::unique_ptr<WholeBodyControllersRigid::WalkingIK> m_IKSolver; /**< Pointer to the inverse
                                                                         kinematics solver. */
    std::unique_ptr<WholeBodyControllersRigid::WalkingQPIK> m_QPIKSolver; /**< Pointer to the
                                                                             inverse kinematics
                                                                             solver. */
    std::unique_ptr<WholeBodyControllersRigid::TaskBasedTorqueControlYARPInterface>
        m_taskBasedTorqueControl; /**< Pointer to
          the task base
          torque
          control. */

    std::unique_ptr<SoftTerrainWalking::WholeBodyControllers::MomentumBasedControl>
        m_momentumBasedController; /**< Pointer to the task base torque control. */
    std::unique_ptr<SoftTerrainWalking::Simulator::Simulator> m_simulator; /**< Pointer to the
                                                                             simulator. */

    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>
        m_desiredFloatingBaseSystemKinematics;
    std::unique_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>
        m_desiredFloatingBaseSystemKinematicsIntegrator; /**< Pointer to the simulator. */
    std::unique_ptr<SoftTerrainWalking::Simulator::Visualizer> m_visualizer; /**< Pointer to the
                                                                               simulator. */

    struct ContactParametersEstimator
    {
        iDynTree::Transform nullForceTransform;
        std::unique_ptr<BipedalLocomotion::Estimators::RecursiveLeastSquare> rls;
        std::unique_ptr<SoftTerrainWalking::ContactModels::ContinuousContactModel> model;
    };

    ContactParametersEstimator m_leftFootEstimator;
    ContactParametersEstimator m_rightFootEstimator;

    std::unique_ptr<WalkingFK> m_FKSolver; /**< Pointer to the forward kinematics solver. */
    std::unique_ptr<TrajectoryPlanner::StableDCMModel> m_stableDCMModel; /**< Pointer to the stable DCM dynamics. */
    std::unique_ptr<WalkingControllers::LoggerClient> m_walkingLogger; /**< Pointer to the Walking Logger object. */
    std::unique_ptr<WalkingControllers::TimeProfiler> m_profiler; /**< Time profiler. */

    double m_additionalRotationWeightDesired; /**< Desired additional rotational weight matrix. */
    double m_desiredJointsWeight; /**< Desired joint weight matrix. */
    yarp::sig::Vector m_desiredJointInRadYarp; /**< Desired joint position (regularization task). */

    std::deque<iDynTree::Transform> m_leftTrajectory; /**< Deque containing the trajectory of the
                                                         left foot. */
    std::deque<iDynTree::Transform> m_rightTrajectory; /**< Deque containing the trajectory of the
                                                          right foot. */

    std::deque<iDynTree::Twist> m_leftTwistTrajectory; /**< Deque containing the twist trajectory of
                                                          the left foot. */
    std::deque<iDynTree::Twist> m_rightTwistTrajectory; /**< Deque containing the twist trajectory
                                                           of the right foot. */

    std::deque<iDynTree::SpatialAcc> m_leftAccelerationTrajectory; /**< Deque containing the
                                                                      acceleration of the left foot.
                                                                    */
    std::deque<iDynTree::SpatialAcc> m_rightAccelerationTrajectory; /**< Deque containing the
                                                                       acceleration of the right
                                                                       foot. */

    std::deque<iDynTree::Vector2> m_DCMPositionDesired; /**< Deque containing the desired DCM
                                                           position. */
    std::deque<iDynTree::Vector2> m_DCMVelocityDesired; /**< Deque containing the desired DCM
                                                           velocity. */
    std::deque<iDynTree::Vector2> m_ZMPPositionDesired; /**< Deque containing the desired ZMP
                                                           position. */
    std::deque<bool> m_leftInContact; /**< Deque containing the left foot state. */
    std::deque<bool> m_rightInContact; /**< Deque containing the right foot state. */
    std::deque<double> m_comHeightTrajectory; /**< Deque containing the CoM height trajectory. */
    std::deque<double> m_comHeightVelocity; /**< Deque containing the CoM height velocity. */
    std::deque<size_t> m_mergePoints; /**< Deque containing the time position of the merge points.
                                       */

    std::deque<bool> m_isLeftFixedFrame; /**< Deque containing when the main frame of the left foot
                                            is the fixed frame In general a main frame of a foot is
                                            the fix frame only during the
                                            stance and the switch out phases. */

    std::deque<double> m_weightInLeft; /**< Deque containing the left foot weight percentage. */
    std::deque<double> m_weightInRight; /**< Deque containing the right foot weight percentage. */

    iDynTree::ModelLoader m_loader; /**< Model loader class. */

    iDynTree::VectorDynSize m_qDesired; /**< Vector containing the results of the IK algorithm
                                           [rad]. */
    iDynTree::VectorDynSize m_dqDesired; /**< Vector containing the results of the IK algorithm
                                            [rad]. */

    iDynTree::Rotation m_inertial_R_worldFrame; /**< Rotation between the inertial and the world
                                                   frame. */

    yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_desiredUnyciclePositionPort; /**< Desired robot
                                                                                position port. */

    bool m_newTrajectoryRequired; /**< if true a new trajectory will be merged soon. (after
                                     m_newTrajectoryMergeCounter - 2 cycles). */
    size_t m_newTrajectoryMergeCounter; /**< The new trajectory will be merged after
                                           m_newTrajectoryMergeCounter - 2 cycles. */

    std::mutex m_mutex; /**< Mutex. */

    iDynTree::Vector2 m_desiredPosition;
    iDynTree::Rotation m_additionalRotation;

    // debug
    std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};

    // spline
    iDynTree::CubicSpline m_weightToComHeightSpline;

    /**
     * Get the robot model from the resource finder and set it.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool setRobotModel(const yarp::os::Searchable& rf);

    /**
     * Propagate time.
     */
    void propagateTime();

    /**
     * Advance the reference signal.
     * @return true in case of success and false otherwise.
     */
    bool advanceReferenceSignals();

    /**
     * Update the FK solver.
     * @return true in case of success and false otherwise.
     */
    bool updateFKSolver();

    /**
     * Set the QP-IK problem.
     * @param solver is the pointer to the solver (osqp or qpOASES)
     * @param desiredCoMPosition desired CoM position;
     * @param desiredCoMVelocity desired CoM velocity;
     * @param desiredNeckOrientation desired neck orientation (rotation matrix);
     * @param output is the output of the solver (i.e. the desired joint velocity)
     * @return true in case of success and false otherwise.
     */
    bool solveQPIK(const std::unique_ptr<WholeBodyControllersRigid::WalkingQPIK>& solver,
                   const iDynTree::Position& desiredCoMPosition,
                   const iDynTree::Vector3& desiredCoMVelocity,
                   const iDynTree::Rotation& desiredNeckOrientation,
                   iDynTree::VectorDynSize& output);

    /**
     * Solve the task based inverse dynamics problem
     * @param desiredCoMPosition desired position of the CoM;
     * @param desiredCoMVelocity desired velocity of the CoM;
     * @param desiredCoMAcceleration desired acceleration of the CoM;
     * @param desiredNeckOrientation desired orientation of the neck.
     */
    void solveTaskBasedInverseDynamics(const iDynTree::Position& desiredCoMPosition,
                                       const iDynTree::Vector3& desiredCoMVelocity,
                                       const iDynTree::Vector3& desiredCoMAcceleration,
                                       const iDynTree::Rotation& desiredNeckOrientation);

    bool solveMomentumBasedWithCompliantContacts(const iDynTree::Position& desiredCoMPosition,
                                                 const iDynTree::Vector3& desiredCoMVelocity,
                                                 const iDynTree::Vector3& desiredCoMAcceleration,
                                                 const iDynTree::Vector3& desiredCoMJerk,
                                                 const iDynTree::Rotation& desiredNeckOrientation);

    bool estimateContactParameters();

    /**
     * Generate the first trajectory.
     * This method has to be called before updateTrajectories() method.
     * @param initialPosition initial position of the robot, If not specified it is assumed to be in
     * the origin
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories(const iDynTree::Position& initialPosition
                                   = iDynTree::Position::Zero());

    /**
     * Generate the first trajectory. (onTheFly)
     * @param leftToRightTransform transformation between left and right feet.
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories(const iDynTree::Transform& leftToRightTransform);

    /**
     * Ask for a new trajectory (The trajectory will be evaluated by a thread).
     * @param initTime is the initial time of the trajectory;
     * @param isLeftSwinging todo wrong name?;
     * @param measuredTransform transformation between the world and the (stance/swing??) foot;
     * @param mergePoint is the instant at which the old and the new trajectory will be merged;
     * @param desiredPosition final desired position of the projection of the CoM.
     * @return true/false in case of success/failure.
     */
    bool askNewTrajectories(const double& initTime,
                            const bool& isLeftSwinging,
                            const iDynTree::Transform& measuredTransform,
                            const size_t& mergePoint,
                            const iDynTree::Vector2& desiredPosition);

    /**
     * Update the old trajectory.
     * This method has to be called only if the trajectory generator has finished to evaluate the
     * new trajectory. The old and the new trajectory will be merged at mergePoint.
     * @param mergePoint instant at which the old and the new trajectory will be merged
     * @return true/false in case of success/failure.
     */
    bool updateTrajectories(const size_t& mergePoint);

    /**
     * Set the input of the planner. The desired position is expressed using a
     * reference frame attached to the robot. The X axis points forward while the
     * Y axis points on the left.
     * @param x desired forward position of the robot
     * @param y desired lateral position of the robot
     * @return true/false in case of success/failure.
     */
    bool setPlannerInput(double x, double y);

    /**
     * Reset the entire controller architecture
     */
    void reset();

public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * This allows you to put the robot in a home position for walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool prepareRobot() override;

    /**
     * Start walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool startWalking() override;
};
}; // namespace WalkingControllers
#endif
