// Collider.cpp
#include "Collider.h"

// Initialize static ID counter
int Collider::s_nextId = 0;

Collider::~Collider() {
    clearExceptions();
}

void Collider::addException(Collider* other) {
    if (!other || other == this) {
        return;
    }
    if (!m_exceptions) {
        m_exceptions = std::make_unique<std::unordered_map<int, Collider*>>();
    }
    if (!other->m_exceptions) {
        other->m_exceptions = std::make_unique<std::unordered_map<int, Collider*>>();
    }
    (*m_exceptions)[other->m_id] = other;
    (*other->m_exceptions)[m_id] = this;
}

void Collider::removeException(Collider* other) {
    if (!other) {
        return;
    }
    if (m_exceptions) {
        m_exceptions->erase(other->m_id);
    }
    if (other->m_exceptions) {
        other->m_exceptions->erase(m_id);
    }
}

void Collider::clearExceptions() {
    if (!m_exceptions) {
        return;
    }
    // Symmetry guarantees every neighbor still holds a live back-edge to this collider,
    // so each neighbor is alive and can be unlinked directly.
    for (const auto& [neighborId, neighbor] : *m_exceptions) {
        if (neighbor->m_exceptions) {
            neighbor->m_exceptions->erase(m_id);
        }
    }
    m_exceptions.reset();
}

void Collider::updatePosition(uint64_t currentTimestep) {
    // Update position if needed and we have a dependent position system
    if (currentTimestep > m_positionValidUntilTime && m_dependentPosition) {
        m_position = m_dependentPosition->m_position + m_dependentPosition->m_orientation * m_dependentOffset;
        m_orientation = m_dependentPosition->m_orientation;
        
        // Mark position as valid for this timestep
        m_positionValidUntilTime = currentTimestep;
    }
}