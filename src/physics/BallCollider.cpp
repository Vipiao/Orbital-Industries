// BallCollider.cpp
#include "BallCollider.h"
#include <glm/gtx/norm.hpp>

BallCollider::BallCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double radius,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_radius(radius)
{
    
}

void BallCollider::updateSimpleAABB() {
    m_AABBMin = m_position - glm::dvec3(m_radius);
    m_AABBMax = m_position + glm::dvec3(m_radius);
}

bool BallCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}