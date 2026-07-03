// SensorCollider.cpp
#include "SensorCollider.h"
#include "utils/GeometryUtils.h"
#include <glm/gtx/norm.hpp>
#include <algorithm>

SensorCollider::SensorCollider(const glm::dvec3& position,
                               const glm::dvec3& halfScale)
    : Collider(position, glm::dquat(1.0, 0.0, 0.0, 0.0))
    , m_halfScale(halfScale)
{
}

void SensorCollider::updateSimpleAABB(uint64_t currentTimestep) {
    // Check if simple AABB is still valid
    if (currentTimestep <= m_simpleAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    // Calculate AABB directly from position and half-scale
    m_AABBMin = m_position - m_halfScale;
    m_AABBMax = m_position + m_halfScale;

    // Mark simple AABB as valid for this timestep
    m_simpleAABBValidUntilTime = currentTimestep;
}

void SensorCollider::updateAdvancedAABB(uint64_t currentTimestep) {
    // Sensors don't need advanced AABB - same as simple
    updateSimpleAABB(currentTimestep);
    m_advancedAABBValidUntilTime = currentTimestep;
}

bool SensorCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

RayIntersectionResult SensorCollider::intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const {
    // Assume ray is already in local space (consistent with other colliders)
    glm::dvec3 localRayStart = rayStart;
    glm::dvec3 localRayEnd = rayEnd;
    
    // AABB slab method
    glm::dvec3 rayDir = localRayEnd - localRayStart;
    double rayLengthSq = glm::length2(rayDir);
    
    if (rayLengthSq < 1e-18) { // 1e-9 squared
        // Degenerate ray - treat as point intersection test
        if (localRayStart.x >= -m_halfScale.x && localRayStart.x <= m_halfScale.x &&
            localRayStart.y >= -m_halfScale.y && localRayStart.y <= m_halfScale.y &&
            localRayStart.z >= -m_halfScale.z && localRayStart.z <= m_halfScale.z) {
            return RayIntersectionResult(0.0, glm::dvec3(0.0)); // Point is inside
        } else {
            return RayIntersectionResult(); // Point is outside
        }
    }
    
    double rayLength = glm::sqrt(rayLengthSq);
    glm::dvec3 rayDirNormalized = rayDir / rayLength;
    
    // Early optimization: if start point is inside AABB, return immediate hit
    if (localRayStart.x >= -m_halfScale.x && localRayStart.x <= m_halfScale.x &&
        localRayStart.y >= -m_halfScale.y && localRayStart.y <= m_halfScale.y &&
        localRayStart.z >= -m_halfScale.z && localRayStart.z <= m_halfScale.z) {
        return RayIntersectionResult(0.0, glm::dvec3(0.0)); // Start inside sensor
    }
    
    double tMin = 0.0;
    double tMax = rayLength;
    
    // Test each axis slab
    for (int i = 0; i < 3; ++i) {
        if (glm::abs(rayDirNormalized[i]) > 1e-9) {
            double t1 = (-m_halfScale[i] - localRayStart[i]) / rayDirNormalized[i];
            double t2 = (m_halfScale[i] - localRayStart[i]) / rayDirNormalized[i];
            
            tMin = glm::max(tMin, glm::min(t1, t2));
            tMax = glm::min(tMax, glm::max(t1, t2));
        } else if (localRayStart[i] < -m_halfScale[i] || localRayStart[i] > m_halfScale[i]) {
            return RayIntersectionResult(); // Ray parallel and outside slab
        }
    }
    
    if (tMin <= tMax && tMin >= 0.0) {
        return RayIntersectionResult(tMin / rayLength, glm::dvec3(0.0)); // Normal calculation skipped for sensors
    }
    
    return RayIntersectionResult();
}