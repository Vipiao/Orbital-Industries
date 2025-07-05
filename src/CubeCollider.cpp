// CubeCollider.cpp
#include "CubeCollider.h"
#include <glm/gtx/transform.hpp>

CubeCollider::CubeCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double width,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_width(width)
{
    
}

void CubeCollider::updateTransformAndAABB() {
    // First update cached vertices
    updateCachedVertices();
    
    // Calculate AABB by finding min/max of cached vertices
    if (m_cachedVertices.empty()) {
        m_AABBMin = m_position - glm::dvec3(m_width * 0.5);
        m_AABBMax = m_position + glm::dvec3(m_width * 0.5);
        return;
    }
    
    m_AABBMin = m_cachedVertices[0];
    m_AABBMax = m_cachedVertices[0];
    
    for (const glm::dvec3& vertex : m_cachedVertices) {
        m_AABBMin = glm::min(m_AABBMin, vertex);
        m_AABBMax = glm::max(m_AABBMax, vertex);
    }
}

bool CubeCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

void CubeCollider::updateCachedVertices() {
    double halfWidth = m_width * 0.5;
    std::vector<glm::dvec3> localVertices = {
        {-halfWidth, -halfWidth, -halfWidth}, { halfWidth, -halfWidth, -halfWidth},
        { halfWidth,  halfWidth, -halfWidth}, {-halfWidth,  halfWidth, -halfWidth},
        {-halfWidth, -halfWidth,  halfWidth}, { halfWidth, -halfWidth,  halfWidth},
        { halfWidth,  halfWidth,  halfWidth}, {-halfWidth,  halfWidth,  halfWidth}
    };
    
    m_cachedVertices.clear();
    m_cachedVertices.reserve(8);
    
    for (const glm::dvec3& localVertex : localVertices) {
        m_cachedVertices.push_back(m_position + m_orientation * localVertex);
    }
}

std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> CubeCollider::getCollisionAxes() const {
    std::vector<glm::dvec3> axes = {
        m_orientation * glm::dvec3(1.0, 0.0, 0.0),  // X-axis
        m_orientation * glm::dvec3(0.0, 1.0, 0.0),  // Y-axis
        m_orientation * glm::dvec3(0.0, 0.0, 1.0)   // Z-axis
    };
    return {axes, axes}; // For cubes, face axes and edge axes are the same
}