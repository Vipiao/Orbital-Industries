// BallCollider.cpp
#include "BallCollider.h"
#include "utils/GeometryUtils.h"
#include <glm/gtx/norm.hpp>

BallCollider::BallCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double radius)
    : Collider(position, orientation)
    , m_radius(radius)
{
    
}

void BallCollider::updateSimpleAABB(uint64_t currentTimestep) {
    // Check if simple AABB is still valid
    if (currentTimestep <= m_simpleAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    m_AABBMin = m_position - glm::dvec3(m_radius);
    m_AABBMax = m_position + glm::dvec3(m_radius);
    m_simpleAABBValidUntilTime = currentTimestep;
}

void BallCollider::updateAdvancedAABB(uint64_t currentTimestep) {
    // For spheres, simple and advanced AABB are identical
    // Just delegate to simple AABB update
    updateSimpleAABB(currentTimestep);
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

RayIntersectionResult BallCollider::intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const {
    RayIntersectionResult result; // Defaults to t=-1, normal=(0,0,0)
    
    // Assume ray is in local space, so sphere is centered at origin
    glm::dvec3 d = rayEnd - rayStart;  // Ray direction * length
    glm::dvec3 f = rayStart;  // Vector from sphere center (origin) to ray start
    
    // Solve quadratic equation: t²*(d·d) + 2*t*(f·d) + (f·f - r²) = 0
    double a = glm::dot(d, d);
    double b = 2.0 * glm::dot(f, d);
    double c = glm::dot(f, f) - m_radius * m_radius;
    
    double discriminant = b * b - 4.0 * a * c;
    
    // No intersection if discriminant is negative
    if (discriminant < 0.0) {
        return result;
    }
    
    // Calculate the two potential intersection parameters
    double sqrtDiscriminant = std::sqrt(discriminant);
    double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
    double t2 = (-b + sqrtDiscriminant) / (2.0 * a);
    
    // Find the closest valid intersection in [0,1]
    double t = -1.0;
    if (t1 >= 0.0 && t1 <= 1.0) {
        t = t1;  // Closer intersection
    } else if (t2 >= 0.0 && t2 <= 1.0) {
        t = t2;  // Further intersection (ray starts inside sphere)
    }
    
    if (t >= 0.0) {
        glm::dvec3 intersectionPoint = rayStart + t * d;
        result.t = t;
        result.surfaceNormal = glm::normalize(intersectionPoint); // Sphere centered at origin
    }
    
    return result;
}