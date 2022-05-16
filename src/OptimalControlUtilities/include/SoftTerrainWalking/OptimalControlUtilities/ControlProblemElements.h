/**
 * @file ControlProblemElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
#define SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <iDynTree/Model/Indices.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <SoftTerrainWalking/OptimalControlUtilities/Frame.h>

namespace SoftTerrainWalking
{

namespace OptimalControlUtilities
{

/**
 * ControlProblemElement describes a general control problem element. The
 * element is considered linear w.r.t the unknown variable.
 * i.e. \f$ A x \f$ where \f$ A \f$ is the element matrix and \f$ x \f$ the
 * unknown variable
 */
class ControlProblemElement
{
protected:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynPtr; /**< KinDyn pointer object */
    iDynTree::MatrixDynSize m_A; /**< Element Matrix */
    std::string m_name; /**< Name of the element */

    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    ControlProblemElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

public:
    /**
     * Get the size of the element (i.e. the number of rows of the element matrix)
     * @return the size of the element
     */
    size_t getSize() const;

    /**
     * Get the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA();

    /**
     * Get the name of the element
     * @return the description of the element
     */
    const std::string& getName() const;
};

/**
 * ControlTask describes a control problem element that will be embedded as cost function or as
 * equality constraint. The element is described by \f$ A \f$ and \f$ b \f$. \f$ A \f$ is the
 * element matrix and \f$ x \f$ and \f$ b \f$ the element vector.
 * In case of cost function \f$ A \f$ and \f$ b \f$ represents:
 * \f$ \|Ax - b\|^2 \f$. In case of equality constraint
 * \f$ Ax = b \f$
 */
class ControlTask : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_b; /**< Element Vector */

    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    ControlTask(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

public:
    /**
     * Get the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB();
};

/**
 * InequalityConstraintElement describes a control problem
 * element that will be embedded as inequality constraint.
 * The element is described by \f$ A \f$, \f$ l \f$ and  \f$ u \f$.
 * \f$ l \le Ax \le u \f$
 */
class InequalityConstraintElement : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_l; /**< Lower bound */
    iDynTree::VectorDynSize m_u; /**< Upper bound */

    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    InequalityConstraintElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

public:

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound();

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound();
};

} // namespace OptimalControlUtilities
} // namespace SoftTerrainWalking

#endif // SOFT_TERRAIN_WALKING_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
