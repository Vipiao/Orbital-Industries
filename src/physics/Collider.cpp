// Collider.cpp
#include "Collider.h"

void Collider::updatePosition(uint64_t currentTimestep) {
    // Update position if needed and we have a dependent position system
    if (currentTimestep > m_positionValidUntilTime && m_dependentPosition) {
        m_position = m_dependentPosition->m_position + m_dependentPosition->m_orientation * m_dependentOffset;
        m_orientation = m_dependentPosition->m_orientation;
        
        // Mark position as valid for this timestep
        m_positionValidUntilTime = currentTimestep;
    }
}