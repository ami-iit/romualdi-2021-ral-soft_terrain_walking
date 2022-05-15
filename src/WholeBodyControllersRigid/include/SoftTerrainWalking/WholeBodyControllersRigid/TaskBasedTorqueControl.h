/**
 * @file TaskBasedTorqueControl.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 Dynamic Interaction Control Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_TASK_BASED_TORQUE_CONTROL
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_TASK_BASED_TORQUE_CONTROL

#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <SoftTerrainWalking/OptimalControlUtilitiesRigid/ControlProblemElements.h>

#include <yarp/os/Searchable.h>

namespace SoftTerrainWalking
{
namespace WholeBodyControllersRigid
{
    class TaskBasedTorqueControl
    {
    protected:
        class Impl;

        std::unique_ptr<Impl> m_pimpl;

        const OptimalControlUtilitiesRigid::VariableHandler * const variableHandler() const;

    public:

        TaskBasedTorqueControl(iDynTree::KinDynComputations& kinDyn);

        ~TaskBasedTorqueControl();

        void setVerbosity(bool isVerbose);

        void addCentroidalLinearMomentumElement(const std::vector<std::string>& framesInContact, bool isConstraint,
                                                const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                                const double& weightScaling = 1,
                                                const double& weightOffset = 0);

        void addCentroidalAngularMomentumElement(const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                                 bool isConstraint,
                                                 const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                                 const double& weightScaling = 1,
                                                 const double& weightOffset = 0);

        void addCartesianElement(const OptimalControlUtilitiesRigid::CartesianElement::Type& type,
                                 const std::string& frameName, const std::string& label,
                                 bool isConstraint,
                                 const OptimalControlUtilitiesRigid::CartesianElement::AxisName& axisName = OptimalControlUtilitiesRigid::CartesianElement::AxisName::X,
                                 const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                 const double& weightScaling = 1,
                                 const double& weightOffset = 0);

        void addSystemDynamicsElement(const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                      bool isConstraint,
                                      const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                      const double& weightScaling = 1,
                                      const double& weightOffset = 0);

        void addSystemDynamicsElement(const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                      const iDynTree::VectorDynSize& gamma,
                                      const iDynTree::VectorDynSize& motorsInertia,
                                      const iDynTree::VectorDynSize& harmonicDriveInertia,
                                      const double& r, const double& R, const double& t,
                                      bool isConstraint,
                                      const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                      const double& weightScaling = 1,
                                      const double& weightOffset = 0);


        void addRegularizationWithControlElement(const std::string& label,
                                                 bool isConstraint,
                                                 const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                                 const double& weightScaling = 1,
                                                 const double& weightOffset = 0);

        void addRegularizationElement(const std::string& label,
                                      bool isConstraint,
                                      const iDynTree::VectorDynSize& weight = iDynTree::VectorDynSize(0),
                                      const double& weightScaling = 1,
                                      const double& weightOffset = 0);

        void addContactWrenchFeasibilityElement(const std::string& variableName, const std::string& frameName,
                                                const int& numberOfPoints, const double& staticFrictionCoefficient,
                                                const double& torsionalFrictionCoefficient, const double& minimalNormalForce,
                                                const iDynTree::Vector2& footLimitX, const iDynTree::Vector2& footLimitY);


        void addJointValuesFeasibilityElement(const std::string& variableName,
                                              const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                              const iDynTree::VectorDynSize& minJointPositionsLimit,
                                              const double& samplingTime);


        OptimalControlUtilitiesRigid::CartesianElement * const  cartesianElement(const std::string& name) const;

        OptimalControlUtilitiesRigid::CentroidalLinearMomentumElement * const centroidalLinearMomentumElement() const;

        OptimalControlUtilitiesRigid::CentroidalAngularMomentumElement * const centroidalAngularMomentumElement() const;

        OptimalControlUtilitiesRigid::SystemDynamicsElement * const systemDynamicsElement() const;

        OptimalControlUtilitiesRigid::RegularizationWithControlElement * const regularizationWithControlElement(const std::string& name) const;

        OptimalControlUtilitiesRigid::RegularizationElement * const regularizationElement(const std::string& name) const;

        OptimalControlUtilitiesRigid::ContactWrenchFeasibilityElement * const contactWrenchFeasibilityElement(const std::string& name) const;

        void initialize();

        void solve();

        bool getVerbosity() const;

        const iDynTree::VectorDynSize& getDesiredTorques();

        iDynTree::Wrench getDesiredWrench(const std::string& name);

        template <typename T, typename U, typename W>
        void setDesiredCartesianTrajectory(const T& acceleration, const U& velocity,
                                           const W& position, const std::string& name)
        {

            OptimalControlUtilitiesRigid::CartesianElement * const element = cartesianElement(name);
            if(element == nullptr)
            {
                if(getVerbosity())
                    std::cerr << "[TaskBasedTorqueControl::setDesiredCartesianTrajectory] The cartesian element called "
                              << name << " is not defined" << std::endl;
                return;
            }


            element->setDesiredTrajectory(acceleration, velocity,
                                          position);
        }

        void setDesiredRegularizationTrajectory(const iDynTree::VectorDynSize& acceleration, const iDynTree::VectorDynSize& velocity,
                                                const iDynTree::VectorDynSize& position, const std::string& name);

        void setContactStateCartesianElement(bool isInContact, const std::string& name);

        void setContactStateWrenchFeasibilityElement(bool isInContact, const std::string& name);

        void setJointState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position);

        void setDesiredVRP(const iDynTree::Vector3& VRP);

        void setDesiredAngularMomentum(const iDynTree::Vector3 &centroidalAngularMomentumVelocity,
                                       const iDynTree::Vector3 &centroidalAngularMomentum);

        void setWeight(const iDynTree::VectorDynSize& weight, const std::string& name);

        void setWeight(const double& weight, const std::string& name);
    };

    class TaskBasedTorqueControlYARPInterface : public TaskBasedTorqueControl
    {
        void addCentroidalLinearMomentumElement(const yarp::os::Searchable &config);

        void addCentroidalAngularMomentumElement(const yarp::os::Searchable &config);

        void addCartesianElement(const yarp::os::Searchable& config, const std::string& label);

        void addSystemDynamicsElement(const yarp::os::Searchable &config);

        void addRegularizationWithControlElement(const yarp::os::Searchable &config, const std::string& label);

        void addRegularizationElement(const yarp::os::Searchable &config, const std::string& label);

        void addContactWrenchFeasibilityElement(const yarp::os::Searchable &config, const std::string& label);

        void addJointValuesFeasibilityElement(const yarp::os::Searchable &config,
                                              const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                              const iDynTree::VectorDynSize& minJointPositionsLimit);

        // Redeclaring scope of base class functions in private section of
        // derived class.
        using TaskBasedTorqueControl::addCentroidalLinearMomentumElement;
        using TaskBasedTorqueControl::addCentroidalAngularMomentumElement;
        using TaskBasedTorqueControl::addCartesianElement;
        using TaskBasedTorqueControl::addSystemDynamicsElement;
        using TaskBasedTorqueControl::addRegularizationWithControlElement;
        using TaskBasedTorqueControl::addRegularizationElement;
        using TaskBasedTorqueControl::addContactWrenchFeasibilityElement;
        using TaskBasedTorqueControl::cartesianElement;
        using TaskBasedTorqueControl::centroidalLinearMomentumElement;
        using TaskBasedTorqueControl::centroidalAngularMomentumElement;
        using TaskBasedTorqueControl::systemDynamicsElement;
        using TaskBasedTorqueControl::regularizationWithControlElement;
        using TaskBasedTorqueControl::regularizationElement;
        using TaskBasedTorqueControl::contactWrenchFeasibilityElement;
        using TaskBasedTorqueControl::addJointValuesFeasibilityElement;

    public:

        TaskBasedTorqueControlYARPInterface(iDynTree::KinDynComputations& kinDyn);

        /**
         * Initialize the Task-based torque control problem.
         * @param config config of the control problem solver
         * @param minJointsPosition is a vector containing the min joints position limit
         * @param minJointsPosition is a vector containing the max joints position limit
         */
        void initialize(const yarp::os::Searchable &config);
    };
}
}
#endif //WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_TASK_BASED_TORQUE_CONTROL
