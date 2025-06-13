// BallCollider.cpp
#include "BallCollider.h"
#include <glm/gtx/norm.hpp>

BallCollider::BallCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double radius,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , radius(radius)
{
    updateCollisionBoxes();
}

CollisionResult BallCollider::collideWith(Collider* other) {
    // Double dispatch: call collideWithBall on other without casting
    return other->collideWithBall(this);
}

void BallCollider::updateCollisionBoxes() {
    collisionBoxMin = position - glm::dvec3(radius);
    collisionBoxMax = position + glm::dvec3(radius);
}

CollisionResult BallCollider::collideWithBall(BallCollider* other) {
    // Calculate distance between centers
    glm::dvec3 direction = other->position - this->position;
    double distanceSquared = glm::length2(direction);
    
    // Calculate sum of radii
    double radiusSum = this->radius + other->radius;
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
        glm::dvec3 contactPoint = this->position + normal * this->radius;
        
        // Calculate penetration depth
        double penetrationDepth = radiusSum - distance;
        
        return CollisionResult(true, normal, contactPoint, penetrationDepth);
    }
    
    // No collision
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

bool BallCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (collisionBoxMin.x <= other->collisionBoxMax.x && 
            collisionBoxMax.x >= other->collisionBoxMin.x &&
            collisionBoxMin.y <= other->collisionBoxMax.y && 
            collisionBoxMax.y >= other->collisionBoxMin.y &&
            collisionBoxMin.z <= other->collisionBoxMax.z && 
            collisionBoxMax.z >= other->collisionBoxMin.z);
}