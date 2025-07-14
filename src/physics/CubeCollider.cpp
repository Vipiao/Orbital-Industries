// CubeCollider.cpp
#include "CubeCollider.h"
#include <glm/gtx/transform.hpp>
#include <algorithm>

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

std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> CubeCollider::getCollisionAxes() const {
    std::vector<glm::dvec3> axes = {
        m_orientation * glm::dvec3(1.0, 0.0, 0.0),  // X-axis
        m_orientation * glm::dvec3(0.0, 1.0, 0.0),  // Y-axis
        m_orientation * glm::dvec3(0.0, 0.0, 1.0)   // Z-axis
    };
    
    // Transform filter normals to world space
    std::vector<glm::dvec3> worldFilterNormals;
    worldFilterNormals.reserve(m_filterNormals.size());
    for (const glm::dvec3& filterNormal : m_filterNormals) {
        worldFilterNormals.push_back(m_orientation * filterNormal);
    }
    
    return {axes, axes, worldFilterNormals}; // For cubes, face axes and edge axes are the same
}

void CubeCollider::addFilterNormal(const glm::dvec3& normal) {
    // Check if this normal already exists (within tolerance)
    const double tolerance = 0.99; // Very close to parallel
    for (const glm::dvec3& existing : m_filterNormals) {
        if (glm::dot(existing, normal) > tolerance) {
            return; // Already exists, don't add duplicate
        }
    }
    
    // Add normalized filter normal
    glm::dvec3 normalizedNormal = glm::normalize(normal);
    m_filterNormals.push_back(normalizedNormal);
}

void CubeCollider::removeFilterNormal(const glm::dvec3& normal) {
    const double tolerance = 0.99; // Very close to parallel
    glm::dvec3 normalizedNormal = glm::normalize(normal);
    
    auto it = std::remove_if(m_filterNormals.begin(), m_filterNormals.end(),
        [&](const glm::dvec3& existing) {
            return glm::dot(existing, normalizedNormal) > tolerance;
        });
    
    m_filterNormals.erase(it, m_filterNormals.end());
}

void CubeCollider::clearFilterNormals() {
    m_filterNormals.clear();
}

const std::vector<glm::dvec3>& CubeCollider::getFilterNormals() const {
    return m_filterNormals;
}