/**
 * @file TaskBasedTorqueControl.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 Dynamic Interaction Control Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <unordered_map>
#include <vector>

#include <OsqpEigen/OsqpEigen.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Wrench.h>

#include <SoftTerrainWalking/WholeBodyControllersRigid/TaskBasedTorqueControl.h>

#include <SoftTerrainWalking/OptimalControlUtilitiesRigid/VariableHandler.h>
#include <SoftTerrainWalking/OptimalControlUtilitiesRigid/OptimizationProblemElements.h>
#include <SoftTerrainWalking/OptimalControlUtilitiesRigid/ControlProblemElements.h>

#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace SoftTerrainWalking::WholeBodyControllersRigid;
using namespace SoftTerrainWalking::OptimalControlUtilitiesRigid;

class TaskBasedTorqueControl::Impl
{
    friend TaskBasedTorqueControl;

    iDynTree::KinDynComputations * const kinDyn; /**< KinDyn pointer object */

    std::unique_ptr<OsqpEigen::Solver> solver;

    std::unique_ptr<VariableHandler> variableHandler;

    // elements that can be used in the controllers
    std::unordered_map<std::string, std::unique_ptr<CartesianElement>> cartesianElements;

    std::unique_ptr<SystemDynamicsElement> systemDynamicsElement;

    std::unique_ptr<CentroidalLinearMomentumElement> centroidalLinearMomentumElement;
    std::unique_ptr<CentroidalAngularMomentumElement> centroidalAngularMomentumElement;

    std::unordered_map<std::string, std::unique_ptr<RegularizationElement>> regularizationElements;
    std::unordered_map<std::string, std::unique_ptr<RegularizationWithControlElement>> regularizationWithControlElements;

    std::unordered_map<std::string, std::unique_ptr<ContactWrenchFeasibilityElement>> contactWrenchFeasibilityElements;

    std::unique_ptr<JointValuesFeasibilityElement> jointValuesFeasibilityElement;

    std::unique_ptr<Constraints> constraints;
    std::unique_ptr<CostFunction> costFunction;

    iDynTree::VectorDynSize jointTorques;

    bool isVerbose;

    bool isSolved{true};

    Impl(iDynTree::KinDynComputations& kinDyn)
        : kinDyn(&kinDyn)
    {
        // instantiate variable handler and initialize the variables
        variableHandler = std::make_unique<VariableHandler>();
        variableHandler->addVariable("base_acceleration", 6);
        variableHandler->addVariable("joint_accelerations", this->kinDyn->model().getNrOfDOFs());
        variableHandler->addVariable("joint_torques", this->kinDyn->model().getNrOfDOFs());
        variableHandler->addVariable("left_foot", 6);
        variableHandler->addVariable("right_foot", 6);

        // initialize the constraints
        constraints = std::make_unique<Constraints>(*variableHandler);

        // initialize the cost function
        costFunction = std::make_unique<CostFunction>(*variableHandler);

        jointTorques.resize(this->kinDyn->model().getNrOfDOFs());
    };

    void initialze()
    {
        // initialize the optimization problem
        solver = std::make_unique<OsqpEigen::Solver>();
        solver->data()->setNumberOfVariables(variableHandler->getNumberOfVariables());
        solver->data()->setNumberOfConstraints(constraints->getNumberOfConstraints());

        solver->settings()->setVerbosity(false);
        solver->settings()->setLinearSystemSolver(0);
        solver->settings()->setMaxIteraction(100000);
        solver->settings()->setPolish(false);

        // print useful information
        std::cout << "Cost Functions" << std::endl;
        std::cout << "---------------------" << std::endl;
        for(const auto& cost : costFunction->getCostFunctions())
            std::cout << cost.second.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;

        std::cout << "Equality Constraints" << std::endl;
        std::cout << "---------------------" << std::endl;
        for(const auto& constraint : constraints->getEqualityConstraints())
            std::cout << constraint.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;

        std::cout << "Inequality Constraints" << std::endl;
        std::cout << "---------------------" << std::endl;
        for(const auto& constraint : constraints->getInequalityConstraints())
            std::cout << constraint.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;
    };

    void solve()
    {
        std::pair<const iDynTree::MatrixDynSize &, iDynTree::VectorDynSize &> costElements = costFunction->getElements();

        Eigen::SparseMatrix<double> hessianSparse = iDynTree::toEigen(costElements.first).sparseView();
        Eigen::SparseMatrix<double> constraintSparse = iDynTree::toEigen(constraints->getConstraintMatrix()).sparseView();
        std::pair<iDynTree::VectorDynSize &, iDynTree::VectorDynSize &>  bounds = constraints->getBounds();
        iDynTree::VectorDynSize & gradient = costElements.second;

        if(solver->isInitialized())
        {
            // update matrices hessian matrix
            // TODO do it in a smart way

            if(!solver->updateHessianMatrix(hessianSparse))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to update the hessian");

            if(!solver->updateGradient(iDynTree::toEigen(gradient)))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to update the gradient");

            if(!solver->updateLinearConstraintsMatrix(constraintSparse))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to update the linear constraint matrix");

            if(!solver->updateBounds(iDynTree::toEigen(bounds.first), iDynTree::toEigen(bounds.second)))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to update the bounds");
        }
        else
        {
            if(!solver->data()->setHessianMatrix(hessianSparse))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to set the hessian the first time");

            if(!solver->data()->setGradient(iDynTree::toEigen(gradient)))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to set the gradient the first time");

            if(!solver->data()->setLinearConstraintsMatrix(constraintSparse))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to set the linear constraint matrix the first time");

            if(!solver->data()->setLowerBound(iDynTree::toEigen(bounds.first)))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to set the lower bounds the first time");

            if(!solver->data()->setUpperBound(iDynTree::toEigen(bounds.second)))
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to set the upper bounds the first time");

            if(!solver->initSolver())
                throw std::runtime_error("[TaskBasedTorqueControlImpl::solve] Unable to initialize the problem");
        }

        // this->isSolved = true;
        if(!solver->solve())
        {
            std::cerr << "unable to solve the problem " << std::endl;
            this->isSolved = false;
        }
    };
};

// TaskBasedTorqueControl
TaskBasedTorqueControl::TaskBasedTorqueControl(iDynTree::KinDynComputations& kinDyn)
    : m_pimpl(new Impl(kinDyn))
{}

TaskBasedTorqueControl::~TaskBasedTorqueControl()
{}

void TaskBasedTorqueControl::setVerbosity(bool isVerbose)
{
    m_pimpl->isVerbose = isVerbose;
}

bool TaskBasedTorqueControl::getVerbosity() const
{
    return m_pimpl->isVerbose;
}

CartesianElement * const TaskBasedTorqueControl::cartesianElement(const std::string& name) const
{
    auto element = m_pimpl->cartesianElements.find(name);

    if(element == m_pimpl->cartesianElements.end())
        return nullptr;

    return element->second.get();
}

CentroidalLinearMomentumElement * const TaskBasedTorqueControl::centroidalLinearMomentumElement() const
{
    return m_pimpl->centroidalLinearMomentumElement.get();
}

CentroidalAngularMomentumElement * const TaskBasedTorqueControl::centroidalAngularMomentumElement() const
{
    return m_pimpl->centroidalAngularMomentumElement.get();
}

SystemDynamicsElement * const TaskBasedTorqueControl::systemDynamicsElement() const
{
    return m_pimpl->systemDynamicsElement.get();
}

RegularizationWithControlElement * const TaskBasedTorqueControl::regularizationWithControlElement(const std::string& name) const
{
    return m_pimpl->regularizationWithControlElements.find(name)->second.get();
}

RegularizationElement * const TaskBasedTorqueControl::regularizationElement(const std::string& name) const
{
    auto element = m_pimpl->regularizationElements.find(name);

    if(element == m_pimpl->regularizationElements.end())
        return nullptr;

    return element->second.get();
}

ContactWrenchFeasibilityElement * const TaskBasedTorqueControl::contactWrenchFeasibilityElement(const std::string& name) const
{
    auto element = m_pimpl->contactWrenchFeasibilityElements.find(name);

    if(element == m_pimpl->contactWrenchFeasibilityElements.end())
        return nullptr;

    return element->second.get();
}

const VariableHandler * const TaskBasedTorqueControl::variableHandler() const
{
    return m_pimpl->variableHandler.get();
}

void TaskBasedTorqueControl::addCentroidalLinearMomentumElement(const std::vector<std::string>& framesInContact, bool isConstraint,
                                                                const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                                const double& weightScaling /*= 1*/,
                                                                const double& weightOffset/*= 0*/)
{
    m_pimpl->centroidalLinearMomentumElement = std::make_unique<CentroidalLinearMomentumElement>(*(m_pimpl->kinDyn),
                                                                                                 *(m_pimpl->variableHandler),
                                                                                                 framesInContact);

    // add to the constraint or to the cost function
    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->centroidalLinearMomentumElement.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->centroidalLinearMomentumElement.get(),
                                               weight, weightScaling, weightOffset,
                                               "centroidal_linear_momentum");
};

void TaskBasedTorqueControl::addCentroidalAngularMomentumElement(const std::vector<std::pair<std::string, std::string>>& framesInContact, bool isConstraint,
                                                                 const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                                 const double& weightScaling /*= 1*/,
                                                                 const double& weightOffset/*= 0*/)
{
    m_pimpl->centroidalAngularMomentumElement = std::make_unique<CentroidalAngularMomentumElement>(*(m_pimpl->kinDyn),
                                                                                                   *(m_pimpl->variableHandler),
                                                                                                   framesInContact);

    // add to the constraint or to the cost function
    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->centroidalAngularMomentumElement.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->centroidalAngularMomentumElement.get(),
                                               weight, weightScaling, weightOffset,
                                               "centroidal_angular_momentum");
};

void TaskBasedTorqueControl::addCartesianElement(const CartesianElement::Type& type,
                                                 const std::string& frameName, const std::string& label,
                                                 bool isConstraint,
                                                 const CartesianElement::AxisName& axisName  /*= CartesianElement::AxisName::X*/,
                                                 const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                 const double& weightScaling /*= 1*/,
                                                 const double& weightOffset/*= 0*/)
{
    if(m_pimpl->cartesianElements.find(label) != m_pimpl->cartesianElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addCartesianElement] The element named "
                                 + label
                                 + " has been already added.");

    m_pimpl->cartesianElements.insert({label,
                std::make_unique<CartesianElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler), type, frameName, axisName)});

    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->cartesianElements.find(label)->second.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->cartesianElements.find(label)->second.get(),
                                               weight, weightScaling, weightOffset,
                                               label + "_cartesian_element");
};

void TaskBasedTorqueControl::addSystemDynamicsElement(const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                                      bool isConstraint,
                                                      const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                      const double& weightScaling /*= 1*/,
                                                      const double& weightOffset/*= 0*/)
{
    m_pimpl->systemDynamicsElement = std::make_unique<SystemDynamicsElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler),
                                                                             framesInContact);

    // add to the constraint or to the cost function
    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->systemDynamicsElement.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->systemDynamicsElement.get(),
                                               weight, weightScaling, weightOffset,
                                               "system_dynamics");
};

void TaskBasedTorqueControl::addSystemDynamicsElement(const std::vector<std::pair<std::string, std::string>>& framesInContact,
                                                      const iDynTree::VectorDynSize& gamma,
                                                      const iDynTree::VectorDynSize& motorsInertia,
                                                      const iDynTree::VectorDynSize& harmonicDriveInertia,
                                                      const double& r, const double& R, const double& t,
                                                      bool isConstraint,
                                                      const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                      const double& weightScaling /*= 1*/,
                                                      const double& weightOffset/*= 0*/)
{
    m_pimpl->systemDynamicsElement = std::make_unique<SystemDynamicsElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler),
                                                                             framesInContact,
                                                                             gamma, motorsInertia, harmonicDriveInertia,
                                                                             r, R, t);

    // add to the constraint or to the cost function
    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->systemDynamicsElement.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->systemDynamicsElement.get(),
                                               weight, weightScaling, weightOffset,
                                               "system_dynamics");
};


void TaskBasedTorqueControl::addRegularizationWithControlElement(const std::string& label,
                                                                 bool isConstraint,
                                                                 const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                                 const double& weightScaling /*= 1*/,
                                                                 const double& weightOffset/*= 0*/)
{
    if(m_pimpl->regularizationWithControlElements.find(label) != m_pimpl->regularizationWithControlElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addRegularizationWithControlElement] This element has been already added.");

    m_pimpl->regularizationWithControlElements.insert({label,
                std::make_unique<RegularizationWithControlElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler), label)});

    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->regularizationWithControlElements.find(label)->second.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->regularizationWithControlElements.find(label)->second.get(),
                                               weight, weightScaling, weightOffset,
                                               label);
}

void TaskBasedTorqueControl::addRegularizationElement(const std::string& label,
                                                      bool isConstraint,
                                                      const iDynTree::VectorDynSize& weight /*= iDynTree::VectorDynSize(0)*/,
                                                      const double& weightScaling /*= 1*/,
                                                      const double& weightOffset/*= 0*/)
{
    if(m_pimpl->regularizationElements.find(label) != m_pimpl->regularizationElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addRegularizationElement] This element has been already added.");

    m_pimpl->regularizationElements.insert({label,
                std::make_unique<RegularizationElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler), label)});

    if(isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->regularizationElements.find(label)->second.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->regularizationElements.find(label)->second.get(),
                                               weight, weightScaling, weightOffset,
                                               label);
}

void TaskBasedTorqueControl::addContactWrenchFeasibilityElement(const std::string& variableName, const std::string& frameName,
                                                                const int& numberOfPoints, const double& staticFrictionCoefficient,
                                                                const double& torsionalFrictionCoefficient, const double& minimalNormalForce,
                                                                const iDynTree::Vector2& footLimitX, const iDynTree::Vector2& footLimitY)
{
    if(m_pimpl->contactWrenchFeasibilityElements.find(variableName) != m_pimpl->contactWrenchFeasibilityElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addContactWrenchFeasibilityElement] This element has been already added.");

    m_pimpl->contactWrenchFeasibilityElements.insert({variableName,
                std::make_unique<ContactWrenchFeasibilityElement>(*(m_pimpl->kinDyn), *(m_pimpl->variableHandler),
                                                                  std::make_pair(variableName, frameName),
                                                                  numberOfPoints, staticFrictionCoefficient,
                                                                  torsionalFrictionCoefficient, minimalNormalForce, footLimitX, footLimitY,
                                                                  OsqpEigen::INFTY)});

    m_pimpl->constraints->addConstraint(m_pimpl->contactWrenchFeasibilityElements.find(variableName)->second.get());
}

void TaskBasedTorqueControl::addJointValuesFeasibilityElement(const std::string& variableName,
                                                              const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                                              const iDynTree::VectorDynSize& minJointPositionsLimit,
                                                              const double& samplingTime)
{
    m_pimpl->jointValuesFeasibilityElement = std::make_unique<JointValuesFeasibilityElement>(*(m_pimpl->kinDyn),
                                                                                             *(m_pimpl->variableHandler),
                                                                                             variableName,
                                                                                             maxJointPositionsLimit,
                                                                                             minJointPositionsLimit,
                                                                                             samplingTime);

    m_pimpl->constraints->addConstraint(m_pimpl->jointValuesFeasibilityElement.get());
}

void TaskBasedTorqueControl::initialize()
{
    m_pimpl->initialze();
}

void TaskBasedTorqueControl::solve()
{
    m_pimpl->solve();
}

const iDynTree::VectorDynSize &TaskBasedTorqueControl::getDesiredTorques() {
  auto jointIndex = m_pimpl->variableHandler->getVariable("joint_torques");

  if (m_pimpl->isSolved)
    iDynTree::toEigen(m_pimpl->jointTorques) =
        m_pimpl->solver->getSolution().segment(jointIndex.offset,
                                               jointIndex.size);
  else
    iDynTree::toEigen(m_pimpl->jointTorques).setZero();
  return m_pimpl->jointTorques;
}

iDynTree::Wrench TaskBasedTorqueControl::getDesiredWrench(const std::string& name)
{
    auto wrenchIndex = m_pimpl->variableHandler->getVariable(name);

    if(!wrenchIndex.isValid())
        return iDynTree::Wrench::Zero();

    iDynTree::Wrench wrench;
    iDynTree::toEigen(wrench.getLinearVec3()) = m_pimpl->solver->getSolution().segment(wrenchIndex.offset, 3);
    iDynTree::toEigen(wrench.getAngularVec3()) = m_pimpl->solver->getSolution().segment(wrenchIndex.offset + 3, 3);

    return wrench;
}

void TaskBasedTorqueControl::setDesiredRegularizationTrajectory(const iDynTree::VectorDynSize& acceleration, const iDynTree::VectorDynSize& velocity,
                                                                const iDynTree::VectorDynSize& position, const std::string& name)
{
    RegularizationWithControlElement * const element = regularizationWithControlElement(name);
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setDesiredRegularizationTrajectory] The regularization element with control called "
                      << name << " is not defined" << std::endl;
        return;
    }

    element->setDesiredTrajectory(acceleration, velocity, position);
}

void TaskBasedTorqueControl::setContactStateCartesianElement(bool isInContact, const std::string& name)
{
    CartesianElement * const element = cartesianElement(name);
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControlInterface::setContactStateCartesianElement] The cartesian element called "
                      << name << " is not defined" << std::endl;
        return;
    }

    element->isInContact(isInContact);
}

void TaskBasedTorqueControl::setContactStateWrenchFeasibilityElement(bool isInContact, const std::string& name)
{
    ContactWrenchFeasibilityElement * const element = contactWrenchFeasibilityElement(name);
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControlInterface::setContactStateWrenchFeasibilityElement] The Contact wrench feasibility element called "
                      << name << " is not defined" << std::endl;
        return;
    }

    element->isInContact(isInContact);
}

void TaskBasedTorqueControl::setJointState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position)
{
    RegularizationWithControlElement * const element = regularizationWithControlElement("joint_accelerations");
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setJointState] The regularization element with control called"
                      << " joint_accelerations is not defined" << std::endl;
        return;
    }

    element->setState(velocity, position);
}

void TaskBasedTorqueControl::setDesiredVRP(const iDynTree::Vector3& VRP)
{
    auto element = centroidalLinearMomentumElement();
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setDesiredVRP] The centroidal linear momentum element does not exist"
                      << std::endl;
        return;
    }

    element->setVRP(VRP);
}

void TaskBasedTorqueControl::setDesiredAngularMomentum(const iDynTree::Vector3 &centroidalAngularMomentumVelocity,
                                                       const iDynTree::Vector3 &centroidalAngularMomentum)
{
    auto element = centroidalAngularMomentumElement();
    if(element == nullptr)
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setDesiredAngularMomentum] The centroidal angular momentum element does not exist"
                      << std::endl;
        return;
    }

    element->setDesiredCentroidalAngularMomentum(centroidalAngularMomentumVelocity, centroidalAngularMomentum);
}

void TaskBasedTorqueControl::setWeight(const iDynTree::VectorDynSize& weight, const std::string& name)
{
    if(!m_pimpl->costFunction->setWeight(weight, name) && m_pimpl->isVerbose)
    {
        std::cerr << "[TaskBasedTorqueControl::setWeight] The cost function element named " << name << " does not exist" << std::endl;
    }
}

void TaskBasedTorqueControl::setWeight(const double& weight, const std::string& name)
{
    if(!m_pimpl->costFunction->setWeight(weight, name) && m_pimpl->isVerbose)
    {
        std::cerr << "[TaskBasedTorqueControl::setWeight] The cost function element named " << name << " does not exist" << std::endl;
    }
}

// TaskBasedTorqueControlYARPInterface
TaskBasedTorqueControlYARPInterface::TaskBasedTorqueControlYARPInterface(iDynTree::KinDynComputations& kinDyn)
    : TaskBasedTorqueControl(kinDyn)
{}

void TaskBasedTorqueControlYARPInterface::addCentroidalLinearMomentumElement(const yarp::os::Searchable &config)
{
    std::vector<std::string> linksInContact{"left_foot", "right_foot"};

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize weight(3);
    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCentroidalLinearMomentumElement] The weight of linear centroidal momentum element cannot be found");


    TaskBasedTorqueControl::addCentroidalLinearMomentumElement(linksInContact, asConstraint, weight);
}

void TaskBasedTorqueControlYARPInterface::addCentroidalAngularMomentumElement(const yarp::os::Searchable &config)
{
    // add the frames in contact
    std::vector<std::pair<std::string, std::string>> framesInContact;

    std::string frameName;
    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "left_foot_frame", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find left_foot_frame");

    framesInContact.push_back({"left_foot", frameName});

    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "right_foot_frame", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find right_foot_frame");

    framesInContact.push_back({"right_foot", frameName});

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize weight(3);
    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCentroidalAngularMomentumElement] The weight of the centroidal angular momentum element cannot be found");

    TaskBasedTorqueControl::addCentroidalAngularMomentumElement(framesInContact, asConstraint, weight);

    double kp;
    if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp", kp))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCentroidalAngularMomentumElement] The kp of centroidal angular momentum element cannot be found");

    centroidalAngularMomentumElement()->setGain(kp);
}

void TaskBasedTorqueControlYARPInterface::addCartesianElement(const yarp::os::Searchable& config, const std::string& label)
{
    CartesianElement::Type type = CartesianElement::Type::POSE;
    CartesianElement::AxisName axis = CartesianElement::AxisName::X;
    std::string frameName;
    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "frame_name", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The frame_name cannot be found");

    std::string typeName;
    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "type", typeName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The type of "
                                 + label
                                 + " cannot be found");

    if(typeName == "position")
        type = CartesianElement::Type::POSITION;
    else if(typeName == "orientation")
        type = CartesianElement::Type::ORIENTATION;
    else if(typeName == "one_dimension")
    {
        type = CartesianElement::Type::ONE_DIMENSION;

        std::string axisName;
        if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "axis_name", axisName))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The axis_name of "
                                     + label
                                     + " cannot be found");

        if(axisName == "Y")
            axis = CartesianElement::AxisName::Y;
        else if(axisName == "Z")
            axis = CartesianElement::AxisName::Z;
    }

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();

    iDynTree::VectorDynSize weight;
    if(type == CartesianElement::Type::POSE)
        weight.resize(6);
    else if(type == CartesianElement::Type::POSITION || type == CartesianElement::Type::ORIENTATION)
        weight.resize(3);
    else if(type == CartesianElement::Type::ONE_DIMENSION)
        weight.resize(1);

    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The weight of "
                                     + label
                                     + " cannot be found");

    TaskBasedTorqueControl::addCartesianElement(type, frameName, label, asConstraint, axis, weight);

    if(type == CartesianElement::Type::POSITION)
    {
        iDynTree::Vector3 kp, kd;
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kp", kp))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp of "
                                     + label
                                     + " cannot be found");

        if(config.check("use_default_kd", yarp::os::Value("False")).asBool())
        {
            double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
            iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
        }
        else
            if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kd", kd))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kd of "
                                         + label
                                         + " cannot be found");
        cartesianElement(label)->setLinearPIDGains(kp, kd);
    }

    else if(type == CartesianElement::Type::ORIENTATION)
    {
        double kp, kd, c0;
        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp", kp))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp of "
                                     + label
                                     + " cannot be found");

        if(config.check("use_default_kd", yarp::os::Value("False")).asBool())
        {
            double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
            kd = 2 / scaling * std::sqrt(kp);
        }
        else
            if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp", kp))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp of "
                                         + label
                                         + " cannot be found");

        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "c0", c0))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The c0 of the "
                                     + label
                                     + " cannot be found");

        cartesianElement(label)->setOrientationPIDGains(c0, kd, kp);
    }

    else if (type == CartesianElement::Type::POSE)
    {
        {
            iDynTree::Vector3 kp, kd;
            if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kp_linear", kp))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp_linear of "
                                         + label
                                         + " cannot be found");

            if(config.check("use_default_kd_linear", yarp::os::Value("False")).asBool())
            {
                double scaling = config.check("scaling_linear", yarp::os::Value(1.0)).asDouble();
                iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
            }
            else
                if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kd_linear", kd))
                    throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kd_linear of "
                                             + label
                                             + " cannot be found");
            cartesianElement(label)->setLinearPIDGains(kp, kd);
        }
        {
            double kp, kd, c0;
            if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp_angular", kp))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp_angular of "
                                         + label
                                         + " cannot be found");

            if(config.check("use_default_kd_angular", yarp::os::Value("False")).asBool())
            {
                double scaling = config.check("scaling_angular", yarp::os::Value(1.0)).asDouble();
                kd = 2 / scaling * std::sqrt(kp);
            }
            else
                if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp_angular", kp))
                    throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp_angular of "
                                             + label
                                             + " cannot be found");

            if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "c0", c0))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The c0 of the "
                                         + label
                                         + " cannot be found");

            cartesianElement(label)->setOrientationPIDGains(c0, kd, kp);
        }
    }

    // if(type == CartesianElement::Type::ONE_DIMENSION)
    else
    {
        double kp, kd;
        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp", kp))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kp of "
                                     + label
                                     + " cannot be found");

        if(config.check("use_default_kd", yarp::os::Value("False")).asBool())
        {
            double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
            kd = 2 / scaling * std::sqrt(kp);
        }
        else
            if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "kp", kp))
                throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The kd of "
                                         + label
                                         + " cannot be found");

        cartesianElement(label)->setOneDegreePIDGains(kp, kd);
    }
}

void TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement(const yarp::os::Searchable &config)
{
    std::vector<std::pair<std::string, std::string>> framesInContact;

    std::string frameName;
    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "left_foot_frame", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find left_foot_frame");

    framesInContact.push_back({"left_foot", frameName});

    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "right_foot_frame", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find right_foot_frame");

    framesInContact.push_back({"right_foot", frameName});

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize weight(this->variableHandler()->getVariable("joint_accelerations").size + 6);
    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The weight of the system dynamics element cannot be found");

    bool useReflectedInertia = config.check("use_reflected_inertia",  yarp::os::Value(false)).asBool();

    if(!useReflectedInertia)
        TaskBasedTorqueControl::addSystemDynamicsElement(framesInContact, asConstraint, weight);
    else
    {
        unsigned int numberOfDoFs = TaskBasedTorqueControl::variableHandler()->getVariable("joint_accelerations").size;
        iDynTree::VectorDynSize gamma(numberOfDoFs);
        iDynTree::VectorDynSize motorsInertia(numberOfDoFs);
        iDynTree::VectorDynSize harmonicDriveInertia(numberOfDoFs);

        double r, R, t;

        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "gamma", gamma))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find gamma");

        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "motors_inertia", motorsInertia))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find motors_inertia");

        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "harmonic_drive_inertia", harmonicDriveInertia))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find harmonic_drive_inertia");

        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "r", r))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find r");

        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "R", R))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find R");

        if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "t", t))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addSystemDynamicsElement] Unable to find t");

        // instantiate the element
        TaskBasedTorqueControl::addSystemDynamicsElement(framesInContact,
                                                         gamma, motorsInertia, harmonicDriveInertia,
                                                         r, R, t,
                                                         asConstraint, weight);
    }
}

void TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement(const yarp::os::Searchable &config,
                                                                              const std::string& label)
{
    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();

    iDynTree::VectorDynSize weight(TaskBasedTorqueControl::variableHandler()->getVariable(label).size);
    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The weight of " + label + " cannot be found");

    TaskBasedTorqueControl::addRegularizationWithControlElement(label, asConstraint, weight);

    // instantiate gains
    iDynTree::VectorDynSize kp(TaskBasedTorqueControl::variableHandler()->getVariable(label).size);
    iDynTree::VectorDynSize kd(TaskBasedTorqueControl::variableHandler()->getVariable(label).size);

    if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kp", kp))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The Kp of the "
                                 + label
                                 + " cannot be found");

    if(config.check("use_default_kd", yarp::os::Value("False")).asBool())
    {
        double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
        iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
    }
    else
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "kd", kd))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The Kd of the "
                                     + label
                                     + " cannot be found");

    // set gains
    regularizationWithControlElement(label)->setPIDGains(kp, kd);
}

void TaskBasedTorqueControlYARPInterface::addRegularizationElement(const yarp::os::Searchable &config, const std::string& label)
{
    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();

    iDynTree::VectorDynSize weight(TaskBasedTorqueControl::variableHandler()->getVariable(label).size);
    weight.zero();

    if(!asConstraint)
        if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "weight", weight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The weight of "
                                     + label
                                     + " cannot be found");

    double weightScaling = config.check("weight_scaling", yarp::os::Value(1.0)).asDouble();
    double weightOffset = config.check("weight_offset", yarp::os::Value(0.0)).asDouble();

    TaskBasedTorqueControl::addRegularizationElement(label, asConstraint, weight, weightScaling, weightOffset);
}

void TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement(const yarp::os::Searchable &config,
                                                                             const std::string& label)
{
    std::string frameName;
    if(!WalkingControllers::YarpUtilities::getStringFromSearchable(config, "frame_name", frameName))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] The  cannot be found");

    double staticFrictionCoefficient;
    if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "static_friction_coefficient", staticFrictionCoefficient))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] static_friction_coefficient of "
                                 + label
                                 + " cannot be found");

    int numberOfPoints;
    if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "number_of_points", numberOfPoints))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] number_of_points of "
                                 + label
                                 + " cannot be found");

    double torsionalFrictionCoefficient;
    if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "torsional_friction_coefficient", torsionalFrictionCoefficient))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] torsional_friction_coefficient of "
                                 + label
                                 + " cannot be found");

    iDynTree::Vector2 footLimitsX;
    if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "foot_limits_x", footLimitsX))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] foot_limits_x of "
                                 + label
                                 + " cannot be found");

    iDynTree::Vector2 footLimitsY;
    if(!WalkingControllers::YarpUtilities::getVectorFromSearchable(config, "foot_limits_y", footLimitsY))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] foot_limits_y of "
                                 + label
                                 + " cannot be found");

    double minimalNormalForce;
    if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "minimal_normal_force", minimalNormalForce))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addContactWrenchFeasibilityElement] foot_limits_y of "
                                 + label
                                 + " cannot be found");

    TaskBasedTorqueControl::addContactWrenchFeasibilityElement(label, frameName, numberOfPoints, staticFrictionCoefficient,
                                                               torsionalFrictionCoefficient, minimalNormalForce,
                                                               footLimitsX, footLimitsY);
}

void TaskBasedTorqueControlYARPInterface::addJointValuesFeasibilityElement(const yarp::os::Searchable &config,
                                                                           const iDynTree::VectorDynSize& maxJointsPosition,
                                                                           const iDynTree::VectorDynSize& minJointsPosition)
{

     double samplingTime;
     if(!WalkingControllers::YarpUtilities::getNumberFromSearchable(config, "sampling_time", samplingTime))
         throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addJointValuesFeasibilityElement] Unable to find the sampling_time");

     TaskBasedTorqueControl::addJointValuesFeasibilityElement("joint_accelerations",
                                                              maxJointsPosition, minJointsPosition,
                                                              samplingTime);
 }

void TaskBasedTorqueControlYARPInterface::initialize(const yarp::os::Searchable &config)
{
    bool isVerbose = config.check("verbosity", yarp::os::Value(false)).asBool();
    TaskBasedTorqueControl::setVerbosity(isVerbose);

    yarp::os::Bottle& comOptions = config.findGroup("COM");
    if(!comOptions.isNull())
        addCartesianElement(comOptions, "CoM");

    yarp::os::Bottle& centroidalLinearMomentumOptions = config.findGroup("CENTROIDAL_LINEAR_MOMEMENTUM");
    if(!centroidalLinearMomentumOptions.isNull())
        addCentroidalLinearMomentumElement(centroidalLinearMomentumOptions);

    yarp::os::Bottle& centroidalAngularMomentumOptions = config.findGroup("CENTROIDAL_ANGULAR_MOMEMENTUM");
    if(!centroidalAngularMomentumOptions.isNull())
        addCentroidalAngularMomentumElement(centroidalAngularMomentumOptions);

    yarp::os::Bottle& leftFootOptions = config.findGroup("LEFT_FOOT");
    if(!leftFootOptions.isNull())
        addCartesianElement(leftFootOptions, "left_foot");

    yarp::os::Bottle& rightFootOptions = config.findGroup("RIGHT_FOOT");
    if(!rightFootOptions.isNull())
        addCartesianElement(rightFootOptions, "right_foot");

    yarp::os::Bottle& torsoOrientationOptions = config.findGroup("TORSO");
    if(!torsoOrientationOptions.isNull())
        addCartesianElement(torsoOrientationOptions, "torso");

    yarp::os::Bottle& systemDynamicsOptions = config.findGroup("SYSTEM_DYNAMICS");
    if(!systemDynamicsOptions.isNull())
        addSystemDynamicsElement(systemDynamicsOptions);

    yarp::os::Bottle& jointRegularizationOptions = config.findGroup("JOINT_REGULARIZATION");
    if(!jointRegularizationOptions.isNull())
        addRegularizationWithControlElement(jointRegularizationOptions, "joint_accelerations");

    yarp::os::Bottle& leftWrenchRegularizationOptions = config.findGroup("LEFT_WRENCH_REGULARIZATION");
    if(!leftWrenchRegularizationOptions.isNull())
        addRegularizationElement(leftWrenchRegularizationOptions, "left_foot");

    yarp::os::Bottle& rightWrenchRegularizationOptions = config.findGroup("RIGHT_WRENCH_REGULARIZATION");
    if(!rightWrenchRegularizationOptions.isNull())
        addRegularizationElement(rightWrenchRegularizationOptions, "right_foot");

    yarp::os::Bottle& jointTorquesOptions = config.findGroup("TORQUE_REGULARIZATION");
    if(!jointTorquesOptions.isNull())
        addRegularizationElement(jointTorquesOptions, "joint_torques");

    yarp::os::Bottle& leftWrenchFeasibilityOptions = config.findGroup("LEFT_WRENCH_FEASIBILITY");
    if(!leftWrenchFeasibilityOptions.isNull())
        addContactWrenchFeasibilityElement(leftWrenchFeasibilityOptions, "left_foot");

    yarp::os::Bottle& rightWrenchFeasibilityOptions = config.findGroup("RIGHT_WRENCH_FEASIBILITY");
    if(!rightWrenchFeasibilityOptions.isNull())
        addContactWrenchFeasibilityElement(rightWrenchFeasibilityOptions, "right_foot");

    // yarp::os::Bottle& jointValuesFeasibilityOptions = config.findGroup("JOINT_VALUES_FEASIBILITY");
    // if(!jointValuesFeasibilityOptions.isNull())
    //     addJointValuesFeasibilityElement(jointValuesFeasibilityOptions, maxJointsPosition, minJointsPosition);

    // initialize the optimization problem
    TaskBasedTorqueControl::initialize();

    return;
}
