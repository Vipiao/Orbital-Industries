// CubeCollider.cpp
#include "CubeCollider.h"
#include <glm/gtx/transform.hpp>
#include <algorithm>
#include <stdexcept>

CubeCollider::CubeCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double width,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_width(width)
{
    
}

void CubeCollider::updateSimpleAABB() {
    // Simple AABB using half diagonal of cube
    double halfDiagonal = m_width * std::sqrt(3.0) * 0.5;
    m_AABBMin = m_position - glm::dvec3(halfDiagonal);
    m_AABBMax = m_position + glm::dvec3(halfDiagonal);

    // Mark cached vertices as dirty since position/orientation may have changed
    m_verticesDirty = true;
    // Mark advanced AABB as dirty since position/orientation may have changed
    m_advancedAABBDirty = true;
    // Mark collision axes as dirty since orientation may have changed
    m_collisionAxesDirty = true;
}

void CubeCollider::updateAdvancedAABB() {
    // Only recalculate if dirty
    if (!m_advancedAABBDirty) {
        return;
    }

    std::vector<glm::dvec3> vertices = getVertices();
    
    if (vertices.empty()) {
        throw std::runtime_error("CubeCollider::updateAdvancedAABB: No vertices available for AABB calculation");
    }
    
    // Initialize with first vertex
    m_AABBMin = vertices[0];
    m_AABBMax = vertices[0];
    
    // Expand to include all vertices
    for (size_t i = 1; i < vertices.size(); ++i) {
        m_AABBMin = glm::min(m_AABBMin, vertices[i]);
        m_AABBMax = glm::max(m_AABBMax, vertices[i]);
    }

    // Mark as clean
    m_advancedAABBDirty = false;
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

std::vector<glm::dvec3> CubeCollider::getVertices() const {
    // Lazy calculation - only recalculate if dirty
    if (m_verticesDirty) {
        updateCachedVertices();
        m_verticesDirty = false;
    }
    return m_cachedVertices;
}

void CubeCollider::updateCachedVertices() const {
    double halfWidth = m_width * 0.5;

    // Pre-allocate vertices array to avoid repeated allocations
    if (m_cachedVertices.size() != 8) {
        m_cachedVertices.resize(8);
    }
    
    static const glm::dvec3 localVertices[8] = {
        {-halfWidth, -halfWidth, -halfWidth}, { halfWidth, -halfWidth, -halfWidth},
        { halfWidth,  halfWidth, -halfWidth}, {-halfWidth,  halfWidth, -halfWidth},
        {-halfWidth, -halfWidth,  halfWidth}, { halfWidth, -halfWidth,  halfWidth},
        { halfWidth,  halfWidth,  halfWidth}, {-halfWidth,  halfWidth,  halfWidth}
    };
    
    for (int i = 0; i < 8; ++i) {
        m_cachedVertices[i] = m_position + m_orientation * localVertices[i];
    }
}

std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> CubeCollider::getCollisionAxes() const {
    // Lazy calculation - only recalculate if dirty
    if (m_collisionAxesDirty) {
        updateCachedCollisionAxes();
        m_collisionAxesDirty = false;
    }
    return m_cachedCollisionAxes;
}

void CubeCollider::updateCachedCollisionAxes() const {
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
    
    m_cachedCollisionAxes = {axes, axes, worldFilterNormals}; // For cubes, face axes and edge axes are the same
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

    // Mark collision axes as dirty since filter normals changed
    m_collisionAxesDirty = true;
}

void CubeCollider::removeFilterNormal(const glm::dvec3& normal) {
    const double tolerance = 0.99; // Very close to parallel
    glm::dvec3 normalizedNormal = glm::normalize(normal);
    
    auto it = std::remove_if(m_filterNormals.begin(), m_filterNormals.end(),
        [&](const glm::dvec3& existing) {
            return glm::dot(existing, normalizedNormal) > tolerance;
        });
    
    m_filterNormals.erase(it, m_filterNormals.end());

    // Mark collision axes as dirty since filter normals changed
    m_collisionAxesDirty = true;
}

void CubeCollider::clearFilterNormals() {
    m_filterNormals.clear();

    // Mark collision axes as dirty since filter normals changed
    m_collisionAxesDirty = true;
}

const std::vector<glm::dvec3>& CubeCollider::getFilterNormals() const {
    return m_filterNormals;
}