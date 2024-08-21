/**
 * @file PDController.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <type_traits>

#include "PIDController.h"

namespace SoftTerrainWalking
{
namespace OptimalControlUtilities
{
template <class T>
PIDController<T>::PIDController(const T& kd, const T& kp, const T& ki)
    : m_kd(kd)
    , m_kp(kp)
    , m_ki(ki)
    , m_controllerOutputEvaluated(false)
{
}

template <class T>
void PIDController<T>::setReference(const T& feedforward,
                                    const T& referenceDerivative,
                                    const T& reference,
                                    const T& referenceIntegral)
{
    m_feedforward = feedforward;
    m_referenceDerivative = referenceDerivative;
    m_reference = reference;
    m_referenceIntegral = referenceIntegral;

    m_controllerOutputEvaluated = false;
}

template <class T> void PIDController<T>::setGains(const T& kd, const T& kp, const T& ki)
{
    m_kd = kd;
    m_kp = kp;
    m_ki = ki;
}

template <class T>
void PIDController<T>::setFeedback(const T& stateDerivative, const T& state, const T& stateIntegral)
{
    m_stateDerivative = stateDerivative;
    m_state = state;
    m_stateIntegral = stateIntegral;

    m_controllerOutputEvaluated = false;
}

template <class T>
const T& PIDController<T>::getControllerOutput()
{
    evaluateControl();
    return m_controllerOutput;
}

template <class T> void PIDController<T>::evaluateControl()
{
    if (this->m_controllerOutputEvaluated)
        return;

    using iDynTree::toEigen;

    toEigen(m_controllerOutput)
        = toEigen(m_feedforward)
          + toEigen(m_kd).asDiagonal()
                * (toEigen(m_referenceDerivative) - toEigen(m_stateDerivative))
          + toEigen(m_kp).asDiagonal() * (toEigen(m_reference) - toEigen(m_state))
          + toEigen(m_ki).asDiagonal() * (toEigen(m_referenceIntegral) - toEigen(m_stateIntegral));

    m_controllerOutputEvaluated = true;
}

} // namespace OptimalControlUtilities
} // namespace SoftTerrainWalking
