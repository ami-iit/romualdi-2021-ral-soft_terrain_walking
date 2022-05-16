/**
 * @file TrajectoryGenerator.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef SOFT_TERRAIN_WALKING_TRAJECTORY_PLANNER_TRAJECTORY_GENERATOR_H
#define SOFT_TERRAIN_WALKING_TRAJECTORY_PLANNER_TRAJECTORY_GENERATOR_H

// std
#include <condition_variable>
#include <memory>
#include <thread>

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>

#include <UnicycleGenerator.h>

namespace SoftTerrainWalking
{

namespace TrajectoryPlanner
{

/**
 * Enumerator useful to track the state of the trajectory generator
 */
enum class GeneratorState
{
    NotConfigured,
    Configured,
    FirstStep,
    Called,
    Returned,
    Closing
};

/**
 * TrajectoryGenerator class is used to handle the UnicycleTrajectoryGenerator library.
 */
class TrajectoryGenerator
{
    UnicycleGenerator m_trajectoryGenerator; /**< UnicycleTrajectoryGenerator object. */
    std::shared_ptr<DCMTrajectoryGenerator> m_dcmGenerator;
    std::shared_ptr<CoMHeightTrajectoryGenerator> m_heightGenerator;
    std::shared_ptr<FeetGenerator> m_feetGenerator;
    bool m_useMinimumJerk;

    bool m_swingLeft; /**< True if the first swing foot is the left. */

    double m_dT; /**< Sampling time of the planner. */
    double m_plannerHorizon; /**< Horizon of the planner. */

    double m_nominalWidth; /**< Nominal width between two feet. */
    double m_initTime; /**< Init time of the current trajectory. */

    iDynTree::Vector2 m_referencePointDistance; /**< Vector between the center of the unicycle and
                                                   the point that has to be reach the goal. */

    GeneratorState m_generatorState{GeneratorState::NotConfigured}; /**< Useful to track the
                                                                       generator state. */

    std::thread m_generatorThread; /**< Main trajectory thread. */
    std::condition_variable m_conditionVariable; /**< Synchronizer. */

    bool m_correctLeft; /**< The left foot has to be corrected. */
    iDynTree::Transform m_measuredTransformLeft; /**< Measured transformation between the left foot
                                                    and the world frame. (w_H_lf) */
    iDynTree::Transform m_measuredTransformRight; /**< Measured transformation between the right
                                                     foot and the world frame. (w_H_rf) */

    iDynTree::Vector2 m_desiredPoint; /**< Desired final position of the x-y projection of the CoM.
                                       */

    iDynTree::Vector2 m_DCMBoundaryConditionAtMergePointPosition; /**< DCM position at the merge
                                                                     point. */
    iDynTree::Vector2 m_DCMBoundaryConditionAtMergePointVelocity; /**< DCM velocity at the merge
                                                                     point. */

    std::mutex m_mutex; /**< Mutex. */

    /**
     * Main thread method.
     */
    void computeThread();

public:
    /**
     * Deconstructor.
     */
    ~TrajectoryGenerator();

    /**
     * Initialize the trajectory generator
     * @param config yarp searchable object.
     * @return true/false in case of success/failure.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Configure the planner.
     * @param config yarp searchable object.
     * @return true/false in case of success/failure.
     */
    bool configurePlanner(const yarp::os::Searchable& config);

    /**
     * Generate the first trajectory.
     * This method has to be called before updateTrajectories() method
     * @param initialPosition initial position of the robot, in case is not specified the position
     * is assumed to be in the origin.
     * @return true/false in case of success/failure.
     */
    bool generateFirstTrajectories(const iDynTree::Position& initialPosition
                                   = iDynTree::Position::Zero());

    /**
     * Generate the first trajectory.
     * This method has to be called before only by the ontTheFly method.
     * @param leftToRightTransform transformation between from the left foot to the right foot;
     * @return true/false in case of success/failure.
     */
    bool generateFirstTrajectories(const iDynTree::Transform& leftToRightTransform);
    // const iDynTree::Position &initialCOMPosition);

    /**
     * Update the trajectory.
     * The old trajectory will be deleted and a new one is evaluated. The boundary condition of the
     * new trajectory is given by the position and the velocity of the DCM at the merge point. This
     * method allows you to take into account the real position one foot at the beginning of the
     * trajectory.
     * @param initTime is the initial time of the trajectory;
     * @param DCMBoundaryConditionAtMergePointPosition is the position of the DCM at the merge
     * point;
     * @param DCMBoundaryConditionAtMergePointVelocity is the velocity of the DCM at the merge
     * point;
     * @param correctLeft todo;
     * @param measured Measured transformation between the stance foot and the world frame.
     * (w_H_{stancefoot});
     * @param desiredPosition final desired position of the projection of the CoM.
     * @return true/false in case of success/failure.
     */
    bool updateTrajectories(double initTime,
                            const iDynTree::Vector2& DCMBoundaryConditionAtMergePointPosition,
                            const iDynTree::Vector2& DCMBoundaryConditionAtMergePointVelocity,
                            bool correctLeft,
                            const iDynTree::Transform& measured,
                            const iDynTree::Vector2& desiredPosition);

    /**
     * Return if the trajectory was computed
     * @return true if the trajectory has been computed false otherwise.
     */
    bool isTrajectoryComputed();

    /**
     * Configure the planner in order to add or not the terminal step
     * @param terminalStep if it true the terminal step will be added
     */
    void addTerminalStep(bool terminalStep);

    /**
     * Return if a new trajectory is asked.
     * @return true if the trajectory has already asked.
     */
    bool isTrajectoryAsked();

    /**
     * Get the desired 2D-DCM position trajectory
     * @param DCMPositionTrajectory desired trajectory of the DCM.
     * @return true/false in case of success/failure.
     */
    bool getDCMPositionTrajectory(std::vector<iDynTree::Vector2>& DCMPositionTrajectory);

    /**
     * Get the desired 2D-DCM velocity trajectory
     * @param DCMVelocityTrajectory desired trajectory of the DCM.
     * @return true/false in case of success/failure.
     */
    bool getDCMVelocityTrajectory(std::vector<iDynTree::Vector2>& DCMVelocityTrajectory);

    /**
     * Get the desired ZMP position trajectory
     * @param ZMPPositionTrajectory desired trajectory of the ZMP.
     * @return true/false in case of success/failure.
     */
    bool getZMPPositionTrajectory(std::vector<iDynTree::Vector2>& ZMPPositionTrajectory);

    /**
     * Get the feet trajectory
     * @param lFootTrajectory vector containing the left foot trajectory;
     * @param rFootTrajectory vector containing the right foot trajectory.
     * @return true/false in case of success/failure.
     */
    bool getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory,
                             std::vector<iDynTree::Transform>& rFootTrajectory);

    /**
     * Get the feet twist
     * @param lFootTwist vector containing the left foot twists;
     * @param rFootTwist vector containing the right foot twists.
     * @return true/false in case of success/failure.
     */
    bool getFeetTwist(std::vector<iDynTree::Twist>& lFootTwist,
                      std::vector<iDynTree::Twist>& rFootTwist);

    /**
     * Get the feet acceleration
     * @param lFootAccleration vector containing the left foot acceleration;
     * @param rFootAcceleration vector containing the right foot acceleration.
     * @return true/false in case of success/failure.
     */
    bool getFeetAcceleration(std::vector<iDynTree::SpatialAcc>& lFootAccleration,
                             std::vector<iDynTree::SpatialAcc>& rFootAcceleration);

    /**
     * Get the when the main frame of the left foot is the fix frame.
     * @param isLeftFixedFrame vector containing when the main frame of
     * the left foot is the fix frame.
     * @return true/false in case of success/failure.
     */
    bool getWhenUseLeftAsFixed(std::vector<bool>& isLeftFixedFrame);

    /**
     * Get the feet phases
     * @param lFootContacts vector containing the state of the left foot (true = in contact);
     * @param rFootContacts vector containing the state of the right foot (true = in contact).
     * @return true/false in case of success/failure.
     */
    bool getFeetStandingPeriods(std::vector<bool>& lFootContacts, std::vector<bool>& rFootContacts);

    /**
     * Get the CoM height trajectory
     * @param CoMHeightTrajectory vector containing trajectory of the COM on the z axis.
     * @return true/false in case of success/failure.
     */
    bool getCoMHeightTrajectory(std::vector<double>& CoMHeightTrajectory);

    /**
     * Get the CoM height velocity
     * @param CoMHeightVelocity vector containing the velocity of the COM on the z axis.
     * @return true/false in case of success/failure.
     */
    bool getCoMHeightVelocity(std::vector<double>& CoMHeightVelocity);

    /**
     * Get the merge points along the trajectory
     * @param mergePoints vector containing all the merge points of the trajectory.
     * @return true/false in case of success/failure.
     */
    bool getMergePoints(std::vector<size_t>& mergePoints);

    /**
     * Get the weight percentage for the left and right foot
     * @param weightInLeft vector containing the weight on the left foot (0 in case in case of
     * stance foot during SS, 1 in case of swing foot)
     * @param weightInRight vector containing the weight on the right foot (0 in case in case of
     * stance foot during SS, 1 in case of swing foot)
     * @return true/false in case of success/failure.
     */
    bool getWeightPercentage(std::vector<double>& weightInLeft, std::vector<double>& weightInRight);

    /**
     * Reset the planner
     */
    void reset();
};
} // namespace TrajectoryPlanner
} // namespace SoftTerrainWalking

#endif
