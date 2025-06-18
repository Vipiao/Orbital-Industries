// BallCollider.cpp
#include "BallCollider.h"
#include "GridCollider.h"
#include <glm/gtx/norm.hpp>

BallCollider::BallCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double radius,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_radius(radius)
{
    
}

CollisionResult BallCollider::collideWith(Collider* other) {
    // Double dispatch: call collideWithBall on other without casting
    return other->collideWithBall(this);
}

void BallCollider::updateAABB() {
    m_AABBMin = m_position - glm::dvec3(m_radius);
    m_AABBMax = m_position + glm::dvec3(m_radius);
}

CollisionResult BallCollider::collideWithBall(BallCollider* other) {
    // Calculate distance between centers
    glm::dvec3 direction = other->m_position - this->m_position;
    double distanceSquared = glm::length2(direction);
    
    // Calculate sum of radii
    double radiusSum = this->m_radius + other->m_radius;
    double radiusSumSquared = radiusSum * radiusSum;
    
    // Check for collision using squared distance test
    if (distanceSquared <= radiusSumSquared) {
        // Collision detected
        double distance = glm::sqrt(distanceSquared);
        
        // Calculate collision normal (from this ball towards other ball)
        glm::dvec3 normal;
        if (distance > 1e-9) { // Avoid division by zero
            normal = direction / distance;
        } else {
            // Balls are at exactly the same position, use arbitrary normal
            normal = glm::dvec3(1.0, 0.0, 0.0);
        }
        
        // Calculate contact point (on the surface of this ball)
        glm::dvec3 contactPoint = this->m_position + normal * this->m_radius;
        
        // Calculate penetration depth
        double penetrationDepth = radiusSum - distance;
        
        return CollisionResult(true, normal, contactPoint, penetrationDepth);
    }
    
    // No collision
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult BallCollider::collideWithGrid(GridCollider* grid) {
    // Delegate to the grid's collision method (reverse the roles)
    return grid->collideWithBall(this);
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