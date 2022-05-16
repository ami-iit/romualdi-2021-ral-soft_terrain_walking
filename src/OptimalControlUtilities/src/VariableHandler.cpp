/**
 * @file VariableHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>
#include <SoftTerrainWalking/OptimalControlUtilities/VariableHandler.h>

using namespace SoftTerrainWalking::OptimalControlUtilities;

bool VariableHandler::addVariable(const std::string& name, const std::size_t& size) noexcept
{
    // if the variable already exist cannot be added again.
    if (m_variables.find(name) != m_variables.end())
    {
        std::cerr << "[VariableHandler::addVariable] The variable name " << name
                  << " already exists";
        return false;
    }

    iDynTree::IndexRange indexRange;
    indexRange.size = size;
    indexRange.offset = m_numberOfVariables;
    m_variables.emplace(name, indexRange);
    m_numberOfVariables += size;

    return true;
}

iDynTree::IndexRange VariableHandler::getVariable(const std::string& name) const noexcept
{
    auto variable = m_variables.find(name);

    // if the variable is present its IndexRange is returned otherwise an
    // invalid IndexRange is provided to the user

    if (variable != m_variables.end())
        return variable->second;
    else
        return iDynTree::IndexRange::InvalidRange();
}

std::size_t VariableHandler::getNumberOfVariables() const noexcept
{
    return m_numberOfVariables;
}
