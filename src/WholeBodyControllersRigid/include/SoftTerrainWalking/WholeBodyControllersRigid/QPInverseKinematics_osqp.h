/**
 * @file QPInverseKinematics_osqp.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_OSQP_H
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_OSQP_H

#include <OsqpEigen/OsqpEigen.h>

#include <SoftTerrainWalking/WholeBodyControllersRigid/QPInverseKinematics.h>

namespace SoftTerrainWalking
{
namespace WholeBodyControllersRigid
{
    class WalkingQPIK_osqp : public WalkingQPIK
    {
        std::unique_ptr<OsqpEigen::Solver> m_optimizerSolver; /**< Optimization solver. */

        /**
         * Set joints velocity bounds
         * @return true/false in case of success/failure.
         */
        virtual void setJointVelocitiesBounds() final;

        /**
         * Initialize the solver
         */
        bool initializeSolver();

        /**
         * Update the solver
         */
        bool updateSolver();

    protected:

        /**
         * Initialize the solver
         */
        virtual void instantiateSolver() final;

        /**
         * Set the number of constraints (it may change according to the solver used)
         */
        virtual void setNumberOfConstraints() final;

        /**
         * Initialize matrices that depends on the solver used
         */
        virtual void initializeSolverSpecificMatrices() final;

    public:
        /**
         * Solve the optimization problem.
         * @return true/false in case of success/failure.
         */
        virtual bool solve() final;
    };
};
}
#endif
