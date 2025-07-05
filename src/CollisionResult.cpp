// CollisionResult.cpp
#include "CollisionResult.h"
#include "Collider.h"

void CollisionResult::calculateLocalContactPoints() {
    if (m_contactPoints.empty()) {
        return;
    }
    
    m_contactPointsLocalA.clear();
    m_contactPointsLocalB.clear();
    m_contactPointsLocalA.reserve(m_contactPoints.size());
    m_contactPointsLocalB.reserve(m_contactPoints.size());
    
    for (const glm::dvec3& worldContact : m_contactPoints) {
        // Transform to local space using collider methods
        glm::dvec3 localA = m_colliderA->worldToLocal(worldContact);
        glm::dvec3 localB = m_colliderB->worldToLocal(worldContact);
        
        m_contactPointsLocalA.push_back(localA);
        m_contactPointsLocalB.push_back(localB);
    }
}